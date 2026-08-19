/**
 *	Copyright 2026 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 */

/*
 * @file canopen.h
 * @date 08/06/2026
 *
 * CANopen host support for the WAM port: SDO client, NMT master, SYNC
 * and heartbeat producers, and the CiA-402 puck configuration sequence.
 * The bus contract is docs/canopen-contract.md in the puck2 repository
 * (v0.3.1); section references below point there.
 *
 * All SDO transactions are serialized under the bus mutex, which also
 * satisfies the contract's one-SDO-client-per-server rule (section 7.6).
 */

#ifndef BARRETT_BUS_CANOPEN_H_
#define BARRETT_BUS_CANOPEN_H_

#include <barrett/bus/abstract/communications_bus.h>

namespace barrett {
namespace bus {

class CANopen {
public:
	// Node identity (contract section 2)
	static const int SAFETY_NODE = 10;
	static const int HOST_NODE = 127;
	static const int MIN_PUCK_NODE = 1;
	static const int MAX_PUCK_NODE = 7;

	// COB-ID allocation (contract section 3)
	static const int COB_NMT        = 0x000;
	static const int COB_SYNC       = 0x080;
	static const int COB_EMCY_BASE  = 0x080;  // + node
	static const int COB_TPDO1_BASE = 0x180;  // + node: puck position / SB status
	static const int COB_QUAD_A     = 0x201;  // torque quad, pucks 1-4
	static const int COB_QUAD_B     = 0x205;  // torque quad, pucks 5-7
	static const int COB_TPDO2_BASE = 0x280;  // + node: puck statusword / SB controlword
	static const int COB_SDO_TX_BASE = 0x580; // + node: SDO server -> host
	static const int COB_SDO_RX_BASE = 0x600; // + node: host -> SDO server
	static const int COB_HB_BASE    = 0x700;  // + node: heartbeat / boot-up
	static const int COB_HB_HOST    = 0x77F;

	// NMT commands (CiA 301)
	static const int NMT_START      = 0x01;
	static const int NMT_STOP       = 0x02;
	static const int NMT_PREOP      = 0x80;
	static const int NMT_RESET_NODE = 0x81;  // NEVER broadcast: the SB would
	                                         // reboot and cut bus power
	static const int NMT_RESET_COMM = 0x82;

	// CiA-402 / P4 objects used by the host (contract section 5.1)
	static const int OD_DEVICE_TYPE   = 0x1000;
	static const int OD_IDENTITY      = 0x1018;  // :03 revision, :04 serial
	static const int OD_SYNC_PERIOD   = 0x1006;  // us
	static const int OD_HB_PRODUCER   = 0x1017;  // ms
	static const int OD_CONTROLWORD   = 0x6040;
	static const int OD_STATUSWORD    = 0x6041;
	static const int OD_MODE_SET      = 0x6060;
	static const int OD_MODE_DISPLAY  = 0x6061;
	static const int OD_POSITION      = 0x6064;
	static const int OD_TARGET_TORQUE = 0x6071;
	static const int OD_RATED_TORQUE  = 0x6076;
	static const int OD_INTERP_PERIOD = 0x60C2;  // :01 value, :02 10^index
	static const int OD_ENC_CONFIG    = 0x3013;  // :01 resolution, :02 UserZero

	static const int MODE_CST = 10;  // cyclic synchronous torque

	// The safety board maps BarrettCAN property p to OD index 0x2000+p
	// (contract section 4.2)
	static const int SB_PROP_BASE = 0x2000;

	static const int SYNC_PERIOD_US = 2000;    // 500 Hz (contract section 8)
	static const int PUCK_HB_PERIOD_MS = 250;  // written to 0x1017 (section 5.3)


	/** Send an NMT command.  node 0 = broadcast (only ever used for
	    NMT_START; a broadcast reset would reboot the safety board and
	    drop bus power to every puck - contract section 10). */
	static int nmt(const CommunicationsBus& bus, int command, int node);

	/** Emit one SYNC (pucks return position TPDOs and apply the
	    previously received torque RPDOs). */
	static int sync(const CommunicationsBus& bus);

	/** Emit one host heartbeat (0x77F; the SB's IDLE-state host-liveness
	    signal, contract section 7.5). */
	static int hostHeartbeat(const CommunicationsBus& bus);

	/** Expedited SDO download (write).  Blocks until the server's
	    response arrives; throws on abort, malformed reply, or timeout.
	    nbytes must be 1, 2, or 4. */
	static void sdoDownload(const CommunicationsBus& bus, int node,
			int index, int sub, int value, int nbytes);

	/** Expedited SDO upload (read).  Returns 0 and fills *value
	    (sign-extended from the object's transferred size) on success;
	    1 on timeout; throws on abort or malformed reply.
	    timeout_s 0.0 = block up to CommunicationsBus::TIMEOUT. */
	static int sdoUpload(const CommunicationsBus& bus, int node,
			int index, int sub, int* value, double timeout_s = 0.0);

	/** Wait for a boot-up (or heartbeat) message from a node.
	    \return 0 on success, 1 on timeout. */
	static int awaitBootUp(const CommunicationsBus& bus, int node, double timeout_s);

	/** Configure one motor puck's communication objects per contract
	    section 5.3: torque-quad RPDO1 slice (dummy mapping), controlword
	    RPDO2 linked to the SB broadcast, sync position TPDO1, statusword
	    TPDO2, heartbeat producer, sync/interpolation periods.  Unused
	    factory PDOs are disabled to protect the bus budget.  Call with
	    the node in NMT pre-operational so no TPDO can fire off a SYNC
	    mid-reconfiguration (BENCH 2026-08-18: a SYNC against a
	    half-configured mapping hard-crashed the P4 app).  Throws on
	    failure.  Mode selection moved to armCstMode(). */
	static void configureMotorPuck(const CommunicationsBus& bus, int node);

	/** Arm CST mode on one motor puck.  BENCH-VERIFIED (2026-08-18,
	    fw 4.3.2): 0x6060 writes are REJECTED (abort 0x05040001) unless
	    the drive is Operation-Enabled, so this briefly walks the CiA-402
	    state machine up (fault-reset, shutdown, enable-operation - safe:
	    mode is still 0/none, so no torque path exists), writes CST,
	    verifies the mode display, and steps back down to
	    Ready-to-switch-on.  Requires motor-bus voltage, i.e. the pendant
	    must be out of E-stop (IDLE); throws otherwise with guidance.
	    BENCH-VERIFY: that the armed mode survives the disable and the
	    SB's later controlword-only activation. */
	static void armCstMode(const CommunicationsBus& bus, int node);

	/** Read the motor rated torque (0x6076, mNm) for torque scaling
	    (contract section 5.2).  Throws if the object is absent or 0. */
	static int readRatedTorque(const CommunicationsBus& bus, int node);

	/** Set a puck's position (zeroing, contract section 5.4) via the
	    manufacturer UserZero object.  VERIFY-ON-BENCH: UserZero is
	    assumed to make position-actual read `counts`; if it turns out
	    to be a subtractive offset, only this function changes. */
	static void definePuckPosition(const CommunicationsBus& bus, int node, int counts);

private:
	static int sdoTransact(const CommunicationsBus& bus, int node,
			const unsigned char* req, unsigned char* reply, double timeout_s);
};


}
}

#endif /* BARRETT_BUS_CANOPEN_H_ */

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
 * The bus contract is docs/canopen-contract.md in the puck2 repository;
 * section references below point there.
 *
 * All SDO transactions are serialized under bus.getSdoMutex(), which also
 * satisfies the contract's one-SDO-client-per-server rule (section 7.6);
 * the cyclic traffic (SYNC, PDOs) only takes the bus I/O mutex.
 */

#ifndef BARRETT_BUS_CANOPEN_H_
#define BARRETT_BUS_CANOPEN_H_

#include <string>
#include <stdint.h>

#include <barrett/bus/abstract/communications_bus.h>

namespace barrett {
namespace bus {

class CANopen {
public:
	// Node identity (contract section 2)
	static const int SAFETY_NODE = 10;
	static const int HOST_NODE = 127;
	static const int MIN_PUCK_NODE = 1;
	static const int MAX_PUCK_NODE = 8;

	// COB-ID allocation (contract section 3)
	static const int COB_NMT        = 0x000;
	static const int COB_SYNC       = 0x080;
	static const int COB_EMCY_BASE  = 0x080;  // + node
	static const int COB_TPDO1_BASE = 0x180;  // + node: puck position / SB status
	static const int COB_QUAD_A     = 0x201;  // torque quad, pucks 1-4
	static const int COB_QUAD_B     = 0x205;  // torque quad, pucks 5-8
	static const int COB_TPDO2_BASE = 0x280;  // + node: puck statusword / SB controlword
	static const int COB_SDO_TX_BASE = 0x580; // + node: SDO server -> host
	static const int COB_SDO_RX_BASE = 0x600; // + node: host -> SDO server
	static const int COB_HB_BASE    = 0x700;  // + node: heartbeat / boot-up
	static const int COB_HB_HOST    = 0x77F;
	static const int HB_BOOT_UP     = 0x00;   // heartbeat state byte of a boot-up

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
	static const int OD_ERROR_CODE    = 0x603F;  // CiA 402 fault code
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
	static const int SB_CANOPEN_VERS = 200;        // first CANopen SB firmware (VERS)
	static const int SB_ZERO_RECORD = 0x2103;      // host zero record (section 4.4)
	static const int SB_ZERO_RECORD_VERS = 209;    // first SB firmware serving it
	static const int OD_RAW_POSITION = 0x3012;     // :01 single-turn encoder, 0-4095

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

	/** Decode an 8-byte expedited upload response: 0 with *value
	    (sign-extended from the transferred size), 1 with *abortCode for
	    an abort, -1 for anything malformed or non-expedited. */
	static int decodeUploadReply(const unsigned char* reply, int* value, uint32_t* abortCode);

	/** Drop replies of this SDO server that nobody is waiting for (late
	    answers to timed-out requests).  Call under bus.getSdoMutex(),
	    right before sending a request. */
	static void drainSdoReplies(const CommunicationsBus& bus, int node);

	/** Discard every heartbeat/boot-up frame of a node that the bus has
	    already buffered.  Call BEFORE an NMT reset so that a frame from
	    the node's previous life cannot pass for its fresh boot-up. */
	static void drainHeartbeats(const CommunicationsBus& bus, int node);

	/** Wait for the boot-up message (heartbeat state HB_BOOT_UP) of a
	    node.  Frames already buffered are examined first, so several
	    nodes reset together can be awaited one after another without
	    losing the boot-ups that arrived out of turn.  Other heartbeat
	    states are ignored (a P4 emits one last Operational heartbeat
	    a few ms after the reset command).  Does not drain first: see
	    drainHeartbeats().  \return 0 on success, 1 on timeout. */
	static int awaitBootUp(const CommunicationsBus& bus, int node, double timeout_s);

	/** Commission the expected and sensor-only node sets on SB r206+.
	    Requires IDLE; throws if either write/readback fails. */
	static void configureSafetyNodes(const CommunicationsBus& bus, int expectedMask, int sensorMask);

	/** The current safety-board WAM model requires 4096 counts/rev on
	    arm nodes 1-4. Check before any drive is reset or enabled. */
	static void validateWamEncoders(const CommunicationsBus& bus);


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

	/** Configure a sensor-only puck (a motorless puck used purely as a
	    joint sensor, e.g. the Alberta wrist's passive J5): position
	    TPDO1 and heartbeat as configureMotorPuck(), but NO torque-quad
	    RPDO and - critically - NO controlword RPDO link, so the SB's
	    enable broadcast can never reach a drive stage with no motor.
	    Such nodes must also be exempted on the SB via 0x2100:04
	    (firmware v203+) and are skipped by armCstMode() in wake. */
	static void configureSensorPuck(const CommunicationsBus& bus, int node);

	/** Which puck nodes are sensor-only (bit n = node n).  Set by
	    ProductManager from the config before waking; Puck::wake() and
	    the SB (via 0x2100:04) both honor it. Default 0 per bus. A WAM8g
	    master and a regular WAM7 slave must not share a process-global mask.
	    Change only during idle startup, before the execution manager runs. */
	static void setSensorOnlyNodes(const CommunicationsBus& bus, int mask) { bus.sensorOnlyNodes = mask; }
	static int getSensorOnlyNodes(const CommunicationsBus& bus) { return bus.sensorOnlyNodes; }
	static bool isSensorOnly(const CommunicationsBus& bus, int node) { return (bus.sensorOnlyNodes >> node) & 1; }

	/** Arm CST mode on one motor puck.  BENCH-VERIFIED (2026-08-18,
	    fw 4.3.2): 0x6060 writes are REJECTED (abort 0x05040001) unless
	    the drive is Operation-Enabled, so this briefly walks the CiA-402
	    state machine up (fault-reset, shutdown, enable-operation - safe
	    only after an NMT reset, which leaves mode 0: no torque path; the
	    target is cleared first as well), writes CST,
	    verifies the mode display, and steps back down to
	    Switch-on-disabled so that only the safety board ever enables
	    torque.  Requires motor-bus voltage, i.e. the pendant must be out
	    of E-stop (IDLE); throws otherwise with guidance.  The armed mode
	    survives the disable (arm-verified 2026-08-19); SB firmware v202+
	    is required, since it verifies CST after its own enable steps. */
	static void armCstMode(const CommunicationsBus& bus, int node);

	/** A drive left armed by an earlier run: mode setting (0x6060) still
	    CST and not Operation-Enabled.  Otherwise false, the reason in *why. */
	static bool cstArmedAndDisabled(const CommunicationsBus& bus, int node, std::string* why);

	/** Read the motor rated torque (0x6076, mNm) for torque scaling
	    (contract section 5.2).  Throws if the object is absent or 0. */
	static int readRatedTorque(const CommunicationsBus& bus, int node);

	/** The WAM's software zero, parked in safety-board RAM (0x2103, SB
	    r209+) so that it survives a program restart.  The board forgets
	    it at boot, on ESTOP->IDLE, and on any puck boot-up or NMT reset,
	    i.e. whenever the pucks' position counters may have restarted.
	    Slot n-1 describes node n; nodeMask says which slots are in use. */
	struct ZeroRecord {
		int nodeMask;              // bit n = node n
		double jp[MAX_PUCK_NODE];  // joint position defined at zeroing, rad
		int raw[MAX_PUCK_NODE];    // 0x6064 at zeroing
		int mech[MAX_PUCK_NODE];   // 0x3012:01 at zeroing
	};
	static void clearZeroRecord(const CommunicationsBus& bus);
	/** Nonzero while the board holds a record (the node mask it covers). */
	static int zeroRecordNodes(const CommunicationsBus& bus);
	static void writeZeroRecord(const CommunicationsBus& bus, const ZeroRecord& record);
	/** Read the record for exactly these nodes, then prove that every
	    one of them kept counting since it was written: heartbeat period
	    still the one wake configured (a reset reverts it), and position
	    counter still in step with the absolute single-turn encoder.
	    \return false, with the reason in *why, if the record is absent,
	    describes other nodes, or no longer fits the pucks. */
	static bool readZeroRecord(const CommunicationsBus& bus, int nodeMask,
			ZeroRecord* record, std::string* why);

	/** A P4's single-turn encoder (0x3012:01) is absolute within one motor
	    turn, which is what lets a WAM be zeroed from a rough home placement.
	    This reads it together with the position counter (0x6064), with the
	    motor at rest.  False if the puck did not answer or was moving. */
	static bool readCounterAndSingleTurn(const CommunicationsBus& bus, int node,
			int* position, int* singleTurn);
	/** Which way the single-turn encoder counts relative to the position
	    counter, from how far each moved between two readings at rest:
	    +1 with it, -1 against it, 0 if the motor turned too little to tell
	    or neither fits (the counter is not tracking the encoder). */
	static int singleTurnDirection(int positionDelta, int singleTurnDelta);

private:
	static int sdoTransact(const CommunicationsBus& bus, int node,
			const unsigned char* req, unsigned char* reply, double timeout_s);

};


}
}

#endif /* BARRETT_BUS_CANOPEN_H_ */

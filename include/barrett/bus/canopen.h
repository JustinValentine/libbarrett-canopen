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
 * CANopen host support for the WAM: SDO client, NMT master, SYNC and
 * heartbeat producers, and the CiA-402 puck configuration sequence.
 * "Contract" below refers to docs/canopen-contract.md in the safety-board
 * firmware repository, which specifies the bus.
 *
 * SDO transactions are serialized under bus.getSdoMutex(); the cyclic
 * traffic (SYNC, PDOs) only takes the bus I/O mutex.
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
	static const int NMT_RESET_NODE = 0x81;  // never broadcast: the safety board
	                                         // would reboot and cut bus power
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

	// RPDO transmission types (P4 manual, table 3)
	static const int RPDO_ON_SYNC = 0;         // applied at the next SYNC
	static const int RPDO_ON_ARRIVAL = 255;    // applied when the frame arrives
	static const int PUCK_HB_PERIOD_MS = 250;  // written to 0x1017 (section 5.3)


	/** Send an NMT command.  node 0 = broadcast; use it only for NMT_START
	    (a broadcast reset would reboot the safety board and drop bus power). */
	static int nmt(const CommunicationsBus& bus, int command, int node);

	/** Emit one SYNC (pucks return position TPDOs and apply the
	    previously received torque RPDOs). */
	static int sync(const CommunicationsBus& bus);

	/** Emit one host heartbeat (0x77F), which the safety board uses to
	    detect a live host while IDLE (contract section 7.5). */
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
	    node.  Buffered frames are examined first, so nodes reset together
	    can be awaited one after another.  Other heartbeat states are
	    ignored.  Does not drain first: see drainHeartbeats().
	    \return 0 on success, 1 on timeout. */
	static int awaitBootUp(const CommunicationsBus& bus, int node, double timeout_s);

	/** Tell the safety board (r206+) which puck nodes it must watch and
	    which of them are sensor-only.  Requires IDLE; throws if either
	    write or its readback fails. */
	static void configureSafetyNodes(const CommunicationsBus& bus, int expectedMask, int sensorMask);

	/** The safety board's velocity model assumes 4096 counts/rev on
	    nodes 1-4.  Throws otherwise. */
	static void validateWamEncoders(const CommunicationsBus& bus);


	/** Configure one motor puck's communication objects (contract section
	    5.3): torque-quad slice on RPDO1 (taken at a SYNC or on arrival,
	    see setTorqueOnArrival()), controlword RPDO2 linked to the
	    safety board's broadcast, position TPDO1 on SYNC, statusword TPDO2,
	    heartbeat producer and the cyclic timing objects.  Unused factory
	    PDOs are disabled.  The node must be in NMT pre-operational: a SYNC
	    against a half-written mapping crashes the P4 firmware.  Throws on
	    failure.  The mode is selected separately, by armCstMode(). */
	static void configureMotorPuck(const CommunicationsBus& bus, int node);

	/** Configure a sensor-only puck (no motor; used as a joint encoder):
	    feedback PDOs and heartbeat as configureMotorPuck(), but no torque
	    RPDO and no controlword link, so the safety board's enable broadcast
	    cannot reach it.  The board must be told as well, see
	    configureSafetyNodes(). */
	static void configureSensorPuck(const CommunicationsBus& bus, int node);

	/** Which puck nodes on this bus are sensor-only (bit n = node n).
	    Set by ProductManager before waking the pucks; do not change it
	    once the execution manager runs. */
	static void setSensorOnlyNodes(const CommunicationsBus& bus, int mask) { bus.sensorOnlyNodes = mask; }
	static int getSensorOnlyNodes(const CommunicationsBus& bus) { return bus.sensorOnlyNodes; }
	static bool isSensorOnly(const CommunicationsBus& bus, int node) { return (bus.sensorOnlyNodes >> node) & 1; }

	/** When the motor pucks take a torque frame (RPDO1): at the next SYNC
	    (false, the contract's setting and the default) or when it arrives
	    (true), which applies it one control period earlier.  Set by
	    ProductManager from bus.torque_on_arrival before waking the pucks;
	    it takes effect when the pucks are next configured. */
	static void setTorqueOnArrival(const CommunicationsBus& bus, bool onArrival) { bus.torqueOnArrival = onArrival; }
	static bool getTorqueOnArrival(const CommunicationsBus& bus) { return bus.torqueOnArrival; }

	/** The time over which a drive moves to a new torque target (0x60C2:01,
	    ms): from 1 to the SYNC period, which is the default and what the P4
	    manual asks for.  Set by ProductManager from
	    bus.torque_interpolation_ms.  Throws if out of range. */
	static void setTorqueInterpolationMs(const CommunicationsBus& bus, int ms);
	static int getTorqueInterpolationMs(const CommunicationsBus& bus) {
		return bus.torqueInterpolationMs > 0 ? bus.torqueInterpolationMs : SYNC_PERIOD_US / 1000;
	}

	/** Select CST mode on one motor puck.  P4 firmware 4.3.2 accepts a
	    0x6060 write only while Operation-Enabled, so this clears the torque
	    target, walks the CiA-402 state machine up, writes and verifies the
	    mode, and steps back down to Switch-on-disabled; the setting
	    survives the disable.  Only call it on a drive that was just reset
	    (mode 0, no torque path).  Needs motor-bus voltage, i.e. the safety
	    board in IDLE; throws otherwise.  On failure it requests an E-stop. */
	static void armCstMode(const CommunicationsBus& bus, int node);

	/** A drive left armed by an earlier run: mode setting (0x6060) still
	    CST and not Operation-Enabled.  Otherwise false, the reason in *why. */
	static bool cstArmedAndDisabled(const CommunicationsBus& bus, int node, std::string* why);

	/** Read the motor rated torque (0x6076, mNm) for torque scaling
	    (contract section 5.2).  Throws if the object is absent or 0. */
	static int readRatedTorque(const CommunicationsBus& bus, int node);

	/** The WAM's software zero, kept in safety-board RAM (0x2103, r209+)
	    so that it survives a program restart.  The board forgets it at
	    boot, on ESTOP->IDLE and on any puck boot-up or NMT reset, i.e.
	    whenever a position counter may have restarted.  Slot n-1 describes
	    node n; nodeMask says which slots are in use. */
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

	/** Read a puck's position counter (0x6064) and its single-turn encoder
	    (0x3012:01, absolute within one motor turn) with the motor at rest.
	    False if the puck did not answer or was moving. */
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

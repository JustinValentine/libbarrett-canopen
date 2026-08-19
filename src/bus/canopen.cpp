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
 * canopen.cpp
 *
 *  Created on: Aug 6, 2026
 */

#include <stdexcept>
#include <cstring>

#include <barrett/os.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/canopen.h>


namespace barrett {
namespace bus {

namespace {

// SDO command specifiers (CiA 301, expedited transfers)
const int SDO_CCS_DOWNLOAD_BASE = 0x23;  // e=1 s=1; | ((4-n) << 2)
const int SDO_CCS_UPLOAD        = 0x40;
const int SDO_SCS_DOWNLOAD_OK   = 0x60;
const int SDO_SCS_UPLOAD_MASK   = 0xE0;
const int SDO_SCS_UPLOAD        = 0x40;
const int SDO_CS_ABORT          = 0x80;

// PDO configuration helpers (CiA 301 COB-ID entries)
const unsigned int PDO_COB_INVALID = 0x80000000u;

// CiA 301 dummy-mapping entry for a 16-bit torque-quad pad.  The
// contract names 0x0006 (UNSIGNED16); 0x0003 (INTEGER16) is what
// Barrett's P3 production configs use in exactly this pattern
// (P335-lcb-2.6.0.csv), so it is the field-proven choice.
const unsigned int PDO_MAP_DUMMY16 = 0x00030010u;

void checkedDownload(const CommunicationsBus& bus, int node, int index, int sub,
		int value, int nbytes)
{
	CANopen::sdoDownload(bus, node, index, sub, value, nbytes);
}

}  // namespace


int CANopen::nmt(const CommunicationsBus& bus, int command, int node)
{
	unsigned char data[2];
	data[0] = command;
	data[1] = node;
	return bus.send(COB_NMT, data, 2);
}

int CANopen::sync(const CommunicationsBus& bus)
{
	// A valid pointer even though the frame is empty: send()
	// unconditionally memcpy()s from it.
	unsigned char dummy = 0;
	return bus.send(COB_SYNC, &dummy, 0);
}

int CANopen::hostHeartbeat(const CommunicationsBus& bus)
{
	unsigned char data[1];
	data[0] = 0x05;  // NMT Operational: the host is the NMT master
	return bus.send(COB_HB_HOST, data, 1);
}

/** Send one 8-byte SDO request and collect the matching 8-byte response
    (index/sub echo verified; stale mismatched responses are discarded).
    \return 0 on success, 1 on timeout. */
int CANopen::sdoTransact(const CommunicationsBus& bus, int node,
		const unsigned char* req, unsigned char* reply, double timeout_s)
{
	BARRETT_SCOPED_LOCK(bus.getMutex());

	int ret = bus.send(COB_SDO_RX_BASE + node, req, 8);
	if (ret != 0) {
		(logMessage("CANopen::%s(): Failed to send SDO request to node %d. "
				"send() returned %d.") % __func__ % node % ret)
				.raise<std::runtime_error>();
	}

	double start = highResolutionSystemTime();
	size_t len;
	while (true) {
		if (timeout_s == 0.0) {
			ret = bus.receive(COB_SDO_TX_BASE + node, reply, len, true);
			if (ret != 0) {
				return 1;  // bus.receive() already timed out/logged
			}
		} else {
			ret = bus.receive(COB_SDO_TX_BASE + node, reply, len, false);
			if (ret == 1) {  // no message yet
				if (highResolutionSystemTime() - start > timeout_s) {
					return 1;
				}
				btsleepRT(0.0002);
				continue;
			} else if (ret != 0) {
				return 1;
			}
		}

		if (len != 8) {
			logMessage("CANopen::%s(): node %d: SDO response with DLC %d, ignored.")
					% __func__ % node % len;
			continue;
		}

		// Match the multiplexer echo; discard stale responses from
		// earlier timed-out transactions.
		if (reply[1] == req[1] && reply[2] == req[2] && reply[3] == req[3]) {
			return 0;
		}
		logMessage("CANopen::%s(): node %d: stale SDO response "
				"(index %02x%02x:%02x, expected %02x%02x:%02x), discarded.")
				% __func__ % node % (int)reply[2] % (int)reply[1] % (int)reply[3]
				% (int)req[2] % (int)req[1] % (int)req[3];
	}
}

void CANopen::sdoDownload(const CommunicationsBus& bus, int node,
		int index, int sub, int value, int nbytes)
{
	if (nbytes != 1 && nbytes != 2 && nbytes != 4) {
		throw std::logic_error("CANopen::sdoDownload(): nbytes must be 1, 2, or 4.");
	}

	unsigned char req[8], reply[8];
	req[0] = SDO_CCS_DOWNLOAD_BASE | ((4 - nbytes) << 2);
	req[1] = index & 0xFF;
	req[2] = (index >> 8) & 0xFF;
	req[3] = sub;
	req[4] = value & 0xFF;
	req[5] = (value >> 8) & 0xFF;
	req[6] = (value >> 16) & 0xFF;
	req[7] = (value >> 24) & 0xFF;

	if (sdoTransact(bus, node, req, reply, 0.0) != 0) {
		(logMessage("CANopen::%s(): node %d: no SDO response for download "
				"of 0x%04x:%02x.") % __func__ % node % index % sub)
				.raise<std::runtime_error>();
	}

	if (reply[0] == SDO_CS_ABORT) {
		int abortCode = reply[4] | ((int)reply[5] << 8)
				| ((int)reply[6] << 16) | ((int)reply[7] << 24);
		(logMessage("CANopen::%s(): node %d: SDO abort 0x%08x writing 0x%04x:%02x.")
				% __func__ % node % abortCode % index % sub)
				.raise<std::runtime_error>();
	}
	if (reply[0] != SDO_SCS_DOWNLOAD_OK) {
		(logMessage("CANopen::%s(): node %d: unexpected SDO response 0x%02x "
				"writing 0x%04x:%02x.") % __func__ % node % (int)reply[0]
				% index % sub).raise<std::runtime_error>();
	}
}

int CANopen::sdoUpload(const CommunicationsBus& bus, int node,
		int index, int sub, int* value, double timeout_s)
{
	unsigned char req[8], reply[8];
	req[0] = SDO_CCS_UPLOAD;
	req[1] = index & 0xFF;
	req[2] = (index >> 8) & 0xFF;
	req[3] = sub;
	req[4] = req[5] = req[6] = req[7] = 0;

	if (sdoTransact(bus, node, req, reply, timeout_s) != 0) {
		return 1;
	}

	if (reply[0] == SDO_CS_ABORT) {
		int abortCode = reply[4] | ((int)reply[5] << 8)
				| ((int)reply[6] << 16) | ((int)reply[7] << 24);
		(logMessage("CANopen::%s(): node %d: SDO abort 0x%08x reading 0x%04x:%02x.")
				% __func__ % node % abortCode % index % sub)
				.raise<std::runtime_error>();
	}
	if ((reply[0] & SDO_SCS_UPLOAD_MASK) != SDO_SCS_UPLOAD  ||  !(reply[0] & 0x02)) {
		(logMessage("CANopen::%s(): node %d: unexpected/non-expedited SDO "
				"response 0x%02x reading 0x%04x:%02x.") % __func__ % node
				% (int)reply[0] % index % sub).raise<std::runtime_error>();
	}

	// Transferred size: n = (cs >> 2) & 3 unused bytes, if indicated
	int nbytes = (reply[0] & 0x01) ? 4 - ((reply[0] >> 2) & 3) : 4;

	// Sign-extend from the transferred size (SB properties are i16/i32;
	// harmless for the small unsigned 402 objects read this way)
	int v = (reply[3 + nbytes] & 0x80) ? -1 : 0;
	for (int i = 3 + nbytes; i >= 4; --i) {
		v = (v << 8) | reply[i];
	}
	*value = v;
	return 0;
}

int CANopen::awaitBootUp(const CommunicationsBus& bus, int node, double timeout_s)
{
	unsigned char data[8];
	size_t len;
	double start = highResolutionSystemTime();

	while (true) {
		int ret = bus.receive(COB_HB_BASE + node, data, len, false);
		if (ret == 0) {
			return 0;  // boot-up or heartbeat: the node is alive
		}
		if (highResolutionSystemTime() - start > timeout_s) {
			return 1;
		}
		btsleepRT(0.01);
	}
}

int CANopen::readRatedTorque(const CommunicationsBus& bus, int node)
{
	int rated = 0;
	if (sdoUpload(bus, node, OD_RATED_TORQUE, 0, &rated) != 0) {
		(logMessage("CANopen::%s(): node %d: no response reading rated torque.")
				% __func__ % node).raise<std::runtime_error>();
	}
	if (rated <= 0) {
		(logMessage("CANopen::%s(): node %d: invalid rated torque %d mNm. "
				"Refusing to derive a torque scale for an unconfigured drive.")
				% __func__ % node % rated).raise<std::runtime_error>();
	}
	return rated;
}

void CANopen::definePuckPosition(const CommunicationsBus& bus, int node, int counts)
{
	sdoDownload(bus, node, OD_ENC_CONFIG, 2, counts, 4);  // UserZero
}

void CANopen::configureMotorPuck(const CommunicationsBus& bus, int node)
{
	if (node < MIN_PUCK_NODE || node > MAX_PUCK_NODE) {
		throw std::logic_error("CANopen::configureMotorPuck(): bad node id.");
	}

	// (Mode selection happens later, in armCstMode(): fw 4.3.2 rejects
	// 0x6060 writes while the drive is disabled - bench 2026-08-18.)

	// 1. RPDO1: this puck's slice of the torque quad (contract 5.3 step
	// 2, layouts in section 6).  CiA 301 procedure: invalidate the COB,
	// clear the map, write entries, set the count, re-link the COB.
	int quadCob = (node <= 4) ? COB_QUAD_A : COB_QUAD_B;
	int slice = (node <= 4) ? (node - 1) : (node - 5);
	checkedDownload(bus, node, 0x1400, 1, PDO_COB_INVALID | quadCob, 4);
	checkedDownload(bus, node, 0x1600, 0, 0, 1);
	for (int j = 0; j < 4; ++j) {
		unsigned int entry = (j == slice) ? 0x60710010u : PDO_MAP_DUMMY16;
		checkedDownload(bus, node, 0x1600, j + 1, entry, 4);
	}
	checkedDownload(bus, node, 0x1600, 0, 4, 1);
	checkedDownload(bus, node, 0x1400, 2, 0, 1);  // type 0: apply on SYNC
	checkedDownload(bus, node, 0x1400, 1, quadCob, 4);

	// 2. RPDO2: controlword, linked to the SB broadcast (0x28A)
	checkedDownload(bus, node, 0x1401, 1, PDO_COB_INVALID | (COB_TPDO2_BASE + SAFETY_NODE), 4);
	checkedDownload(bus, node, 0x1601, 0, 0, 1);
	checkedDownload(bus, node, 0x1601, 1, 0x60400010, 4);
	checkedDownload(bus, node, 0x1601, 0, 1, 1);
	checkedDownload(bus, node, 0x1401, 2, 255, 1);  // apply immediately
	checkedDownload(bus, node, 0x1401, 1, COB_TPDO2_BASE + SAFETY_NODE, 4);

	// 3. TPDO1: position actual, transmitted on every SYNC
	checkedDownload(bus, node, 0x1800, 1, PDO_COB_INVALID | (COB_TPDO1_BASE + node), 4);
	checkedDownload(bus, node, 0x1A00, 0, 0, 1);
	checkedDownload(bus, node, 0x1A00, 1, 0x60640020, 4);
	checkedDownload(bus, node, 0x1A00, 0, 1, 1);
	checkedDownload(bus, node, 0x1800, 2, 0, 1);  // every SYNC
	checkedDownload(bus, node, 0x1800, 1, COB_TPDO1_BASE + node, 4);

	// 4. TPDO2: statusword + mode display, every 5th SYNC (10 ms at the
	// 500 Hz cycle).  Sync-cadenced transmission is the bench-proven
	// mechanism on fw 4.3.2 (type 255 + inhibit/event-timer is
	// unverified there), and 10 ms comfortably feeds the SB's 100 ms
	// activation-step timeout.
	checkedDownload(bus, node, 0x1801, 1, PDO_COB_INVALID | (COB_TPDO2_BASE + node), 4);
	checkedDownload(bus, node, 0x1A01, 0, 0, 1);
	checkedDownload(bus, node, 0x1A01, 1, 0x60410010, 4);
	checkedDownload(bus, node, 0x1A01, 2, 0x60610008, 4);
	checkedDownload(bus, node, 0x1A01, 0, 2, 1);
	checkedDownload(bus, node, 0x1801, 2, 5, 1);
	checkedDownload(bus, node, 0x1801, 1, COB_TPDO2_BASE + node, 4);

	// 5. Disable the remaining factory PDOs (velocity/position targets
	// and extra feedback) so they cannot eat into the section 8 bus
	// budget.
	checkedDownload(bus, node, 0x1402, 1, PDO_COB_INVALID | (0x400 + node), 4);
	checkedDownload(bus, node, 0x1403, 1, PDO_COB_INVALID | (0x500 + node), 4);
	checkedDownload(bus, node, 0x1802, 1, PDO_COB_INVALID | (0x380 + node), 4);
	checkedDownload(bus, node, 0x1803, 1, PDO_COB_INVALID | (0x480 + node), 4);

	// 6. Heartbeat producer
	checkedDownload(bus, node, OD_HB_PRODUCER, 0, PUCK_HB_PERIOD_MS, 2);

	// 7. Cyclic timing objects for CST (BENCH 2026-08-18: with these
	// unset, mode-10 CST accepted torque targets but never acted on
	// them - "the need to init CyclicSync parameters" from the design
	// review is real).  0x1006 is the communication cycle period in us;
	// 0x60C2 is the interpolation time period, value * 10^index seconds
	// (2 * 10^-3 = 2 ms = the 500 Hz cycle).  Rejections are logged but
	// non-fatal so a firmware that lacks one object still brings up;
	// if CST then refuses to act, this is the first place to look.
	try {
		checkedDownload(bus, node, OD_SYNC_PERIOD, 0, SYNC_PERIOD_US, 4);
	} catch (const std::runtime_error&) {
		logMessage("CANopen::%s(): node %d: 0x1006 write rejected (non-fatal).")
				% __func__ % node;
	}
	try {
		checkedDownload(bus, node, OD_INTERP_PERIOD, 1, SYNC_PERIOD_US / 1000, 1);
		checkedDownload(bus, node, OD_INTERP_PERIOD, 2, 0xFD, 1);  // 10^-3
	} catch (const std::runtime_error&) {
		logMessage("CANopen::%s(): node %d: 0x60C2 write rejected (non-fatal).")
				% __func__ % node;
	}
}

void CANopen::armCstMode(const CommunicationsBus& bus, int node)
{
	// Walk the CiA-402 state machine up to Operation-Enabled.  Safe with
	// no mode selected (mode 0): the torque path does not exist yet, and
	// the SB's controlword broadcast is quiescent outside of mode
	// transitions.  Requires motor-bus voltage (pendant out of E-stop).
	checkedDownload(bus, node, OD_CONTROLWORD, 0, 0x0080, 2);  // fault reset
	btsleepRT(0.05);
	checkedDownload(bus, node, OD_CONTROLWORD, 0, 0x0006, 2);  // shutdown
	btsleepRT(0.05);
	checkedDownload(bus, node, OD_CONTROLWORD, 0, 0x000F, 2);  // enable op
	double start = highResolutionSystemTime();
	int sw = 0;
	while (true) {
		if (sdoUpload(bus, node, OD_STATUSWORD, 0, &sw, 0.1) == 0
				&& (sw & 0x006F) == 0x0027) {  // Operation enabled
			break;
		}
		if (highResolutionSystemTime() - start > 1.0) {
			(logMessage("CANopen::%s(): node %d: drive did not reach "
					"Operation-Enabled (statusword 0x%04x).  Is the pendant "
					"out of E-stop?  Motor-bus voltage is required to arm "
					"CST mode.") % __func__ % node % sw)
					.raise<std::runtime_error>();
		}
		btsleepRT(0.02);
	}

	// Mode writes are only accepted in this state (fw 4.3.2, bench)
	checkedDownload(bus, node, OD_MODE_SET, 0, MODE_CST, 1);
	int modeDisplay = 0;
	start = highResolutionSystemTime();
	while (true) {
		if (sdoUpload(bus, node, OD_MODE_DISPLAY, 0, &modeDisplay, 0.1) == 0
				&& modeDisplay == MODE_CST) {
			break;
		}
		if (highResolutionSystemTime() - start > 0.5) {
			(logMessage("CANopen::%s(): node %d: mode display is %d, expected "
					"%d (CST).") % __func__ % node % modeDisplay % MODE_CST)
					.raise<std::runtime_error>();
		}
		btsleepRT(0.01);
	}

	// Step back down; the SB's activation sequence owns enabling from
	// here (contract section 7.1)
	checkedDownload(bus, node, OD_CONTROLWORD, 0, 0x0006, 2);  // shutdown
}


}
}

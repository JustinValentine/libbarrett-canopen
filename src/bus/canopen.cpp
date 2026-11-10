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

#include <algorithm>
#include <stdexcept>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>

#include <barrett/os.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/canopen.h>


namespace barrett {
namespace bus {

/* Out-of-class definitions: pre-C++17, ODR-used static const members
   (e.g. passed by const reference into std::vector::push_back) need
   these or the link fails. */
const int CANopen::SAFETY_NODE;
const int CANopen::HOST_NODE;
const int CANopen::MIN_PUCK_NODE;
const int CANopen::MAX_PUCK_NODE;
const int CANopen::COB_NMT;
const int CANopen::COB_SYNC;
const int CANopen::COB_EMCY_BASE;
const int CANopen::COB_TPDO1_BASE;
const int CANopen::COB_QUAD_A;
const int CANopen::COB_QUAD_B;
const int CANopen::COB_TPDO2_BASE;
const int CANopen::COB_SDO_TX_BASE;
const int CANopen::COB_SDO_RX_BASE;
const int CANopen::COB_HB_BASE;
const int CANopen::HB_BOOT_UP;
const int CANopen::COB_HB_HOST;
const int CANopen::NMT_START;
const int CANopen::NMT_STOP;
const int CANopen::NMT_PREOP;
const int CANopen::NMT_RESET_NODE;
const int CANopen::NMT_RESET_COMM;
const int CANopen::OD_DEVICE_TYPE;
const int CANopen::OD_IDENTITY;
const int CANopen::OD_SYNC_PERIOD;
const int CANopen::OD_HB_PRODUCER;
const int CANopen::OD_CONTROLWORD;
const int CANopen::OD_STATUSWORD;
const int CANopen::OD_ERROR_CODE;
const int CANopen::OD_MODE_SET;
const int CANopen::OD_MODE_DISPLAY;
const int CANopen::OD_POSITION;
const int CANopen::OD_TARGET_TORQUE;
const int CANopen::OD_RATED_TORQUE;
const int CANopen::OD_INTERP_PERIOD;
const int CANopen::OD_ENC_CONFIG;
const int CANopen::MODE_CST;
const int CANopen::SB_PROP_BASE;
const int CANopen::SB_CANOPEN_VERS;
const int CANopen::SB_ZERO_RECORD;
const int CANopen::SB_ZERO_RECORD_VERS;
const int CANopen::OD_RAW_POSITION;
const int CANopen::SYNC_PERIOD_US;
const int CANopen::PUCK_HB_PERIOD_MS;

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
	BARRETT_SCOPED_LOCK(bus.getSdoMutex());

	// Anything already buffered is a late reply to an earlier, timed-out
	// transaction; left there, it would answer this request.
	drainSdoReplies(bus, node);
	int ret = bus.send(COB_SDO_RX_BASE + node, req, 8);
	if (ret != 0) {
		(logMessage("CANopen::%s(): Failed to send SDO request to node %d. "
				"send() returned %d.") % __func__ % node % ret)
				.raise<std::runtime_error>();
	}

	const double deadline = highResolutionSystemTime()
			+ (timeout_s > 0.0 ? timeout_s : CommunicationsBus::TIMEOUT);
	size_t len;
	while (true) {
		// The deadline covers malformed/stale traffic too. A continuous
		// stream of wrong replies must not trap an enabled-drive setup.
		if (highResolutionSystemTime() >= deadline) return 1;
		ret = bus.receive(COB_SDO_TX_BASE + node, reply, len, false);
		if (ret == 1) {
			btsleepRT(0.0002);
			continue;
		} else if (ret != 0) {
			return 1;
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
		uint32_t abortCode = uint32_t(reply[4]) | (uint32_t(reply[5]) << 8)
				| (uint32_t(reply[6]) << 16) | (uint32_t(reply[7]) << 24);
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

	uint32_t abortCode = 0;
	switch (decodeUploadReply(reply, value, &abortCode)) {
	case 0:
		return 0;
	case 1:
		(logMessage("CANopen::%s(): node %d: SDO abort 0x%08x reading 0x%04x:%02x.")
				% __func__ % node % abortCode % index % sub)
				.raise<std::runtime_error>();
	default:
		(logMessage("CANopen::%s(): node %d: unexpected/non-expedited SDO "
				"response 0x%02x reading 0x%04x:%02x.") % __func__ % node
				% (int)reply[0] % index % sub).raise<std::runtime_error>();
	}
	return 1;  // not reached
}

int CANopen::decodeUploadReply(const unsigned char* reply, int* value, uint32_t* abortCode)
{
	if (reply[0] == SDO_CS_ABORT) {
		*abortCode = uint32_t(reply[4]) | (uint32_t(reply[5]) << 8)
				| (uint32_t(reply[6]) << 16) | (uint32_t(reply[7]) << 24);
		return 1;
	}
	if ((reply[0] & SDO_SCS_UPLOAD_MASK) != SDO_SCS_UPLOAD  ||  !(reply[0] & 0x02)
			|| (reply[0] & 0x10) || (!(reply[0] & 1) && (reply[0] & 0x0C))) {
		return -1;
	}

	// Transferred size: n = (cs >> 2) & 3 unused bytes, if indicated
	int nbytes = (reply[0] & 0x01) ? 4 - ((reply[0] >> 2) & 3) : 4;

	// Sign-extend from the transferred size (SB properties are i16/i32;
	// harmless for the small unsigned 402 objects read this way)
	uint32_t v = (reply[3 + nbytes] & 0x80) ? UINT32_MAX : 0;
	for (int i = 3 + nbytes; i >= 4; --i) {
		v = (v << 8) | reply[i];
	}
	int32_t signedValue;
	std::memcpy(&signedValue, &v, sizeof(signedValue));
	*value = signedValue;
	return 0;
}

void CANopen::drainSdoReplies(const CommunicationsBus& bus, int node)
{
	// Bounded: a real backlog is a late reply or two.  A server that
	// keeps talking is left to the transaction's own deadline.
	unsigned char data[8];
	size_t len;
	for (int n = 0; n < 8  &&  bus.receive(COB_SDO_TX_BASE + node, data, len, false) == 0; ++n) {
		logMessage("CANopen: node %d: late SDO reply 0x%02x%02x:%02x dropped.")
				% node % (int)data[2] % (int)data[1] % (int)data[3];
	}
}

void CANopen::drainHeartbeats(const CommunicationsBus& bus, int node)
{
	unsigned char data[8];
	size_t len;
	while (bus.receive(COB_HB_BASE + node, data, len, false) == 0) {
	}
}

int CANopen::awaitBootUp(const CommunicationsBus& bus, int node, double timeout_s)
{
	unsigned char data[8];
	size_t len;

	// No drain here.  Puck::wake() resets all pucks in one burst and
	// their boot-ups land within a few ms of each other, long before the
	// later pucks' turn to be awaited; draining at that point threw the
	// boot-up away and timed out (opus start-1433.log, 2026-09-22).
	// Stale frames from before the reset are the caller's business:
	// drainHeartbeats() before sending the reset.
	double start = highResolutionSystemTime();
	while (true) {
		while (bus.receive(COB_HB_BASE + node, data, len, false) == 0) {
			if (len == 1  &&  data[0] == HB_BOOT_UP) {
				return 0;
			}
			// Any other state is a heartbeat, not proof of the reboot.
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


namespace {
// 0x2103 layout (contract section 4.4): sub 1 = key, subs 2-9 = joint
// position of nodes 1-8 in micro-radians, 10-17 = 0x6064, 18-25 = 0x3012:01.
const int ZERO_KEY_MAGIC = 0x5A000000;  // | node mask; 0 = no record
const int ZERO_SUB_KEY = 1, ZERO_SUB_JP = 2, ZERO_SUB_RAW = 10, ZERO_SUB_MECH = 18;
const int SINGLE_TURN_COUNTS = 4096;
const int ZERO_SLIP_COUNTS = 16;  // counter vs. single-turn encoder, 0.4% of a turn

bool validZeroNodes(int nodeMask)
{
	return nodeMask > 0 && (nodeMask & ~0x01FE) == 0;
}

int readOrThrow(const CommunicationsBus& bus, int node, int index, int sub)
{
	int value = 0;
	if (CANopen::sdoUpload(bus, node, index, sub, &value) != 0) {
		(logMessage("node %d did not answer reading 0x%04x:%02x") % node % index % sub)
				.raise<std::runtime_error>();
	}
	return value;
}

// A count difference folded into one turn: -2048..2047.
int withinTurn(int counts)
{
	int r = counts % SINGLE_TURN_COUNTS;
	if (r >= SINGLE_TURN_COUNTS / 2) r -= SINGLE_TURN_COUNTS;
	if (r < -SINGLE_TURN_COUNTS / 2) r += SINGLE_TURN_COUNTS;
	return r;
}

// Has this puck counted continuously since the record was written?  A
// reset restores its saved heartbeat period (wake configures another)
// and restarts 0x6064 at 0 wherever the rotor happens to be, so the
// counter's offset from the absolute single-turn encoder moves too.  The
// encoder's direction relative to 0x6064 is not assumed.
bool keptCounting(const CommunicationsBus& bus, int node, int raw0, int mech0, std::string* why)
{
	char text[160];
	const int hb = readOrThrow(bus, node, CANopen::OD_HB_PRODUCER, 0);
	if (hb != CANopen::PUCK_HB_PERIOD_MS) {
		snprintf(text, sizeof(text), "puck %d was reset (heartbeat period %d ms)", node, hb);
		*why = text;
		return false;
	}
	if (readOrThrow(bus, node, CANopen::OD_ENC_CONFIG, 1) != SINGLE_TURN_COUNTS) {
		snprintf(text, sizeof(text), "puck %d does not count %d per turn", node, SINGLE_TURN_COUNTS);
		*why = text;
		return false;
	}
	const int before = readOrThrow(bus, node, CANopen::OD_POSITION, 0);
	const int mech = readOrThrow(bus, node, CANopen::OD_RAW_POSITION, 1);
	const int after = readOrThrow(bus, node, CANopen::OD_POSITION, 0);
	if (std::abs(after - before) > ZERO_SLIP_COUNTS) {
		snprintf(text, sizeof(text), "puck %d is moving", node);
		*why = text;
		return false;
	}
	const int counted = before - raw0, turned = withinTurn(mech - mech0);
	if (std::min(std::abs(withinTurn(counted - turned)), std::abs(withinTurn(counted + turned)))
			> ZERO_SLIP_COUNTS) {
		snprintf(text, sizeof(text), "puck %d's position counter (moved %d counts) "
				"disagrees with its single-turn encoder (moved %d)", node, counted, turned);
		*why = text;
		return false;
	}
	return true;
}
}  // namespace

void CANopen::clearZeroRecord(const CommunicationsBus& bus)
{
	sdoDownload(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_KEY, 0, 4);
}

int CANopen::zeroRecordNodes(const CommunicationsBus& bus)
{
	const int key = readOrThrow(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_KEY);
	return ((key & ~0x01FE) == ZERO_KEY_MAGIC) ? (key & 0x01FE) : 0;
}

void CANopen::writeZeroRecord(const CommunicationsBus& bus, const ZeroRecord& record)
{
	if (!validZeroNodes(record.nodeMask)) {
		throw std::invalid_argument("CANopen::writeZeroRecord(): bad node mask.");
	}
	// Key last: a record interrupted half way stays empty.
	clearZeroRecord(bus);
	for (int node = MIN_PUCK_NODE; node <= MAX_PUCK_NODE; ++node) {
		if ( !(record.nodeMask & (1 << node)) ) {
			continue;
		}
		const double microRad = record.jp[node - 1] * 1e6;
		if ( !(std::fabs(microRad) < 2e9) ) {  // also rejects NaN
			throw std::invalid_argument("CANopen::writeZeroRecord(): joint position out of range.");
		}
		sdoDownload(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_JP + node - 1, (int)std::lround(microRad), 4);
		sdoDownload(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_RAW + node - 1, record.raw[node - 1], 4);
		sdoDownload(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_MECH + node - 1, record.mech[node - 1], 4);
	}
	sdoDownload(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_KEY, ZERO_KEY_MAGIC | record.nodeMask, 4);
}

bool CANopen::readZeroRecord(const CommunicationsBus& bus, int nodeMask,
		ZeroRecord* record, std::string* why)
{
	try {
		const int stored = zeroRecordNodes(bus);
		if (stored == 0) {
			*why = "no zero is stored";
			return false;
		}
		if (stored != nodeMask) {
			*why = "the stored zero belongs to a different set of pucks";
			return false;
		}
		record->nodeMask = nodeMask;
		for (int node = MIN_PUCK_NODE; node <= MAX_PUCK_NODE; ++node) {
			if (nodeMask & (1 << node)) {
				record->jp[node - 1] = 1e-6 * readOrThrow(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_JP + node - 1);
				record->raw[node - 1] = readOrThrow(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_RAW + node - 1);
				record->mech[node - 1] = readOrThrow(bus, SAFETY_NODE, SB_ZERO_RECORD, ZERO_SUB_MECH + node - 1);
			}
		}
		if (zeroRecordNodes(bus) != nodeMask) {  // cleared while being read
			*why = "the stored zero was cleared while it was being read";
			return false;
		}
		for (int node = MIN_PUCK_NODE; node <= MAX_PUCK_NODE; ++node) {
			if ((nodeMask & (1 << node))
					&& !keptCounting(bus, node, record->raw[node - 1], record->mech[node - 1], why)) {
				return false;
			}
		}
	} catch (const std::exception& e) {
		*why = e.what();
		return false;
	}
	return true;
}

bool CANopen::readCounterAndSingleTurn(const CommunicationsBus& bus, int node,
		int* position, int* singleTurn)
{
	int before = 0, mech = 0, after = 0;
	if (sdoUpload(bus, node, OD_POSITION, 0, &before) != 0
			|| sdoUpload(bus, node, OD_RAW_POSITION, 1, &mech) != 0
			|| sdoUpload(bus, node, OD_POSITION, 0, &after) != 0
			|| std::abs(after - before) > ZERO_SLIP_COUNTS) {
		return false;
	}
	*position = before + (after - before) / 2;
	*singleTurn = mech;
	return true;
}

int CANopen::singleTurnDirection(int positionDelta, int singleTurnDelta)
{
	const int with = std::abs(withinTurn(singleTurnDelta - positionDelta));
	const int against = std::abs(withinTurn(singleTurnDelta + positionDelta));
	if (with <= ZERO_SLIP_COUNTS  &&  against > 4 * ZERO_SLIP_COUNTS) {
		return 1;
	}
	if (against <= ZERO_SLIP_COUNTS  &&  with > 4 * ZERO_SLIP_COUNTS) {
		return -1;
	}
	return 0;
}

void CANopen::configureSafetyNodes(const CommunicationsBus& bus, int expectedMask, int sensorMask)
{
	if (expectedMask <= 0 || (expectedMask & ~0x01FE) != 0
			|| sensorMask < 0 || (sensorMask & ~expectedMask) != 0) {
		throw std::invalid_argument("CANopen::configureSafetyNodes(): invalid expected/sensor node masks.");
	}
	int value = 0;
	// Legacy property MODE is number 8; the SB maps it to 0x2008.
	if (sdoUpload(bus, SAFETY_NODE, SB_PROP_BASE + 8, 0, &value) != 0 || value != 1) {
		throw std::runtime_error("CANopen::configureSafetyNodes(): safety board must be IDLE before startup.");
	}
	sdoDownload(bus, SAFETY_NODE, 0x2100, 4, sensorMask, 2);
	if (sdoUpload(bus, SAFETY_NODE, 0x2100, 4, &value) != 0 || value != sensorMask) {
		throw std::runtime_error("CANopen::configureSafetyNodes(): sensor-only mask readback failed.");
	}
	sdoDownload(bus, SAFETY_NODE, 0x2100, 5, expectedMask, 2);
	if (sdoUpload(bus, SAFETY_NODE, 0x2100, 5, &value) != 0 || value != expectedMask) {
		throw std::runtime_error("CANopen::configureSafetyNodes(): required node mask readback failed; SB r206+ is required.");
	}
}

namespace {
void validateWamEncoder(const CommunicationsBus& bus, int node)
{
	int counts = 0;
	if (CANopen::sdoUpload(bus, node, CANopen::OD_ENC_CONFIG, 1, &counts) != 0 || counts != 4096) {
		(logMessage("CANopen: WAM node %d reports %d encoder counts/rev; the safety board requires 4096. Startup stopped.")
				% node % counts).raise<std::runtime_error>();
	}
}
}

void CANopen::validateWamEncoders(const CommunicationsBus& bus)
{
	for (int node = 1; node <= 4; ++node) validateWamEncoder(bus, node);
}

namespace {
// Contract 5.3 steps 3-6, the part every puck on the WAM bus gets: sync
// position TPDO1, statusword TPDO2, the factory PDOs switched off, and the
// heartbeat producer.
void configureFeedback(const CommunicationsBus& bus, int node)
{
	// 3. TPDO1: position actual, transmitted on every SYNC
	CANopen::sdoDownload(bus, node, 0x1800, 1, PDO_COB_INVALID | (CANopen::COB_TPDO1_BASE + node), 4);
	CANopen::sdoDownload(bus, node, 0x1A00, 0, 0, 1);
	CANopen::sdoDownload(bus, node, 0x1A00, 1, 0x60640020, 4);
	CANopen::sdoDownload(bus, node, 0x1A00, 0, 1, 1);
	CANopen::sdoDownload(bus, node, 0x1800, 2, 0, 1);  // every SYNC
	CANopen::sdoDownload(bus, node, 0x1800, 1, CANopen::COB_TPDO1_BASE + node, 4);

	// 4. TPDO2: statusword + mode display, sync-cadenced (type 5).
	// BENCH 2026-09-16 (fw 4.3.2, odin 4-DOF): the drive emits this PDO
	// every 100 ms whatever the type value (0, 1, 2 and 5 all measured
	// 100.0 ms) and reports each 402 state transition ~100 ms after its
	// controlword; the P4 manual lists inhibit time and event timer as
	// not implemented, so type 255 cannot give a fixed cadence either.
	// SB r208 sizes its activation step timeout (400 ms) and statusword
	// stale bound (350 ms) for that.  Position TPDO1 with type 0 does
	// run every SYNC (2 ms).
	CANopen::sdoDownload(bus, node, 0x1801, 1, PDO_COB_INVALID | (CANopen::COB_TPDO2_BASE + node), 4);
	CANopen::sdoDownload(bus, node, 0x1A01, 0, 0, 1);
	CANopen::sdoDownload(bus, node, 0x1A01, 1, 0x60410010, 4);
	CANopen::sdoDownload(bus, node, 0x1A01, 2, 0x60610008, 4);
	CANopen::sdoDownload(bus, node, 0x1A01, 0, 2, 1);
	CANopen::sdoDownload(bus, node, 0x1801, 2, 5, 1);
	CANopen::sdoDownload(bus, node, 0x1801, 1, CANopen::COB_TPDO2_BASE + node, 4);

	// 5. Disable the remaining factory PDOs (velocity/position targets
	// and extra feedback) so they cannot eat into the section 8 bus
	// budget.
	CANopen::sdoDownload(bus, node, 0x1402, 1, PDO_COB_INVALID | (0x400 + node), 4);
	CANopen::sdoDownload(bus, node, 0x1403, 1, PDO_COB_INVALID | (0x500 + node), 4);
	CANopen::sdoDownload(bus, node, 0x1802, 1, PDO_COB_INVALID | (0x380 + node), 4);
	CANopen::sdoDownload(bus, node, 0x1803, 1, PDO_COB_INVALID | (0x480 + node), 4);

	// 6. Heartbeat producer
	CANopen::sdoDownload(bus, node, CANopen::OD_HB_PRODUCER, 0, CANopen::PUCK_HB_PERIOD_MS, 2);
}
}  // namespace

void CANopen::configureMotorPuck(const CommunicationsBus& bus, int node)
{
	if (node < MIN_PUCK_NODE || node > MAX_PUCK_NODE) {
		throw std::logic_error("CANopen::configureMotorPuck(): bad node id.");
	}

	// Re-check after reset: RAM settings may differ from saved settings.
	if (node <= 4) validateWamEncoder(bus, node);

	// (Mode selection happens later, in armCstMode(): fw 4.3.2 rejects
	// 0x6060 writes while the drive is disabled - bench 2026-08-18.)

	// 1. RPDO1: this puck's slice of the torque quad (contract 5.3 step
	// 2, layouts in section 6).  CiA 301 procedure: invalidate the COB,
	// clear the map, write entries, set the count, re-link the COB.
	int quadCob = (node <= 4) ? COB_QUAD_A : COB_QUAD_B;
	int slice = (node <= 4) ? (node - 1) : (node - 5);
	sdoDownload(bus, node, 0x1400, 1, PDO_COB_INVALID | quadCob, 4);
	sdoDownload(bus, node, 0x1600, 0, 0, 1);
	for (int j = 0; j < 4; ++j) {
		unsigned int entry = (j == slice) ? 0x60710010u : PDO_MAP_DUMMY16;
		sdoDownload(bus, node, 0x1600, j + 1, entry, 4);
	}
	sdoDownload(bus, node, 0x1600, 0, 4, 1);
	sdoDownload(bus, node, 0x1400, 2, 0, 1);  // type 0: apply on SYNC
	sdoDownload(bus, node, 0x1400, 1, quadCob, 4);

	// 2. RPDO2: controlword + modes-of-operation, linked to the SB
	// broadcast (0x28A).  SB v202 broadcasts 3 bytes: controlword +
	// CST mode.  fw 4.3.2 self-disables an enabled drive after ~55
	// SYNCs without a cyclic controlword refresh (the SB streams it
	// every 50 ms while ACTIVE), and carrying the mode byte with every
	// refresh guarantees a 0x6060 write lands while Operation-Enabled -
	// the only time the mode actually engages.
	sdoDownload(bus, node, 0x1401, 1, PDO_COB_INVALID | (COB_TPDO2_BASE + SAFETY_NODE), 4);
	sdoDownload(bus, node, 0x1601, 0, 0, 1);
	sdoDownload(bus, node, 0x1601, 1, 0x60400010, 4);
	sdoDownload(bus, node, 0x1601, 2, 0x60600008, 4);
	sdoDownload(bus, node, 0x1601, 0, 2, 1);
	sdoDownload(bus, node, 0x1401, 2, 255, 1);  // apply immediately
	sdoDownload(bus, node, 0x1401, 1, COB_TPDO2_BASE + SAFETY_NODE, 4);

	configureFeedback(bus, node);  // 3-6

	// P4 4.3.2 requires both cyclic timing objects for CST. A rejected
	// setting cannot be treated as a successful configuration.
	sdoDownload(bus, node, OD_SYNC_PERIOD, 0, SYNC_PERIOD_US, 4);
	sdoDownload(bus, node, OD_INTERP_PERIOD, 1, SYNC_PERIOD_US / 1000, 1);
	sdoDownload(bus, node, OD_INTERP_PERIOD, 2, 0xFD, 1);
}

void CANopen::configureSensorPuck(const CommunicationsBus& bus, int node)
{
	if (node < MIN_PUCK_NODE || node > MAX_PUCK_NODE) {
		throw std::logic_error("CANopen::configureSensorPuck(): bad node id.");
	}

	// A motorless sensing puck (the wam8g's passive J5).  Position TPDO1
	// and statusword TPDO2 as configureMotorPuck(), so it rides the same
	// SYNC and satisfies the SB's per-puck liveness rule - but BOTH
	// command RPDOs stay unlinked: no torque-quad slice, and above all
	// no controlword link, so neither the SB's enable broadcast nor a
	// stray quad can ever reach a drive stage that has no motor.
	sdoDownload(bus, node, 0x1400, 1, PDO_COB_INVALID | (0x200 + node), 4);
	sdoDownload(bus, node, 0x1401, 1, PDO_COB_INVALID | (0x300 + node), 4);

	configureFeedback(bus, node);  // 3-6

	// (No cyclic-timing or mode objects: nothing cyclic is commanded.)
}

void CANopen::armCstMode(const CommunicationsBus& bus, int node)
{
	try {
		// Clear a persisted/previous target before temporarily enabling CST.
		sdoDownload(bus, node, OD_TARGET_TORQUE, 0, 0, 2);

		// Walk the CiA-402 state machine up to Operation-Enabled.  Safe with
		// no mode selected (mode 0, as an NMT reset leaves it - Puck::wake()
		// never arms a drive it did not just reset): the torque path does not
		// exist yet, and the SB's controlword broadcast is quiescent outside
		// of mode transitions.  Requires motor-bus voltage (pendant out of
		// E-stop).
		sdoDownload(bus, node, OD_CONTROLWORD, 0, 0x0080, 2);  // fault reset
		btsleepRT(0.05);
		// A fault that survives the reset (typically 0x3220 under-voltage:
		// motor bus not powered) leaves the drive in Fault, from which
		// "shutdown" is not a legal transition; fw 4.3.2 then refuses the
		// write with the cryptic abort 0x05040001 (opus 2026-09-22,
		// gravcal-1858.log).  Say what is actually wrong instead.
		{
			int sw = 0, fault = 0;
			if (sdoUpload(bus, node, OD_STATUSWORD, 0, &sw, 0.2) == 0
					&& (sw & 0x004F) == 0x0008) {
				sdoUpload(bus, node, OD_ERROR_CODE, 0, &fault, 0.2);
				const char* what = (fault == 0x3220) ? "under-voltage: the motor bus is not powered"
						: (fault == 0x3210) ? "over-voltage" : "see the drive's fault code";
				(logMessage("CANopen::%s(): node %d: fault 0x%04x persists after "
						"fault reset (%s).  Check that the pendant is Idle, the "
						"motor-bus supply is on, and the bus voltage at the arm.")
						% __func__ % node % fault % what).raise<std::runtime_error>();
			}
		}
		sdoDownload(bus, node, OD_CONTROLWORD, 0, 0x0006, 2);  // shutdown
		btsleepRT(0.05);
		sdoDownload(bus, node, OD_CONTROLWORD, 0, 0x000F, 2);  // enable op
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
		sdoDownload(bus, node, OD_MODE_SET, 0, MODE_CST, 1);
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

		// Step back down to Switch-on-disabled (controlword 0 = disable
		// voltage).  Only the safety board may enable torque (contract 7.1);
		// the drive was enabled here solely because fw 4.3.2 accepts 0x6060
		// in no other state, and the mode SETTING survives the disable
		// (arm-verified 2026-08-19).  The mode DISPLAY reads 0 while
		// disabled, which is why SB firmware v202+ verifies CST AFTER its
		// own enable steps and carries the mode byte in every controlword
		// broadcast.  (SB v200/v201 gated on the display before enabling and
		// could only activate host-enabled drives - upgrade the SB rather
		// than leaving drives enabled outside the gatekeeper: between here
		// and the first ~55 SYNCs a host-enabled drive would act on any
		// nonzero torque quad.)
		sdoDownload(bus, node, OD_CONTROLWORD, 0, 0x0000, 2);
		start = highResolutionSystemTime();
		while (true) {
			if (sdoUpload(bus, node, OD_STATUSWORD, 0, &sw, 0.1) == 0
					&& (sw & 0x004F) == 0x0040) {  // Switch-on-disabled (bit 5 is "don't care" here)
				break;
			}
			if (highResolutionSystemTime() - start > 0.5) {
				(logMessage("CANopen::%s(): node %d: drive did not report "
						"Switch-on-disabled after arming CST (statusword 0x%04x). Startup stopped.")
						% __func__ % node % sw).raise<std::runtime_error>();
			}
			btsleepRT(0.01);
		}
	} catch (...) {
		try {
			// Request bus cutoff before waiting again on a failed drive.
			sdoDownload(bus, SAFETY_NODE, SB_PROP_BASE + 8, 0, 0, 2);
		} catch (...) {
			logMessage("CANopen::armCstMode(): safety-board ESTOP request failed during cleanup.", true);
		}
		try {
			sdoDownload(bus, node, OD_CONTROLWORD, 0, 0, 2);
		} catch (...) {
			logMessage("CANopen::armCstMode(): failed to disable node during cleanup.", true);
		}
		throw;
	}

}

bool CANopen::cstArmedAndDisabled(const CommunicationsBus& bus, int node, std::string* why)
{
	int mode = 0, sw = 0;
	try {
		if (sdoUpload(bus, node, OD_MODE_SET, 0, &mode) != 0
				|| sdoUpload(bus, node, OD_STATUSWORD, 0, &sw) != 0) {
			*why = "no reply reading its mode and statusword";
			return false;
		}
	} catch (const std::exception& e) {
		*why = e.what();
		return false;
	}
	char text[80];
	if (mode != MODE_CST) {
		snprintf(text, sizeof(text), "mode setting %d, not CST", mode);
		*why = text;
		return false;
	}
	if ((sw & 0x006F) == 0x0027) {
		*why = "drive is Operation-Enabled";
		return false;
	}
	return true;
}


}
}

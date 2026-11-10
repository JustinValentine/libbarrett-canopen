/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
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
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */

/*
 * puck.cpp
 *
 *  Created on: Aug 23, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <string>
#include <vector>

#include <barrett/os.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/canopen.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>


namespace barrett {


bool Puck::propToOd(int id, int propId, int* index, int* sub, int* nbytes)
{
	if (propId < 0) {
		return false;
	}

	if (id == bus::CANopen::SAFETY_NODE) {
		// Contract section 4.2: every r197 property, 0x2000 + p.
		// PEN (43) is the only 32-bit safety-board property; ADDR (6),
		// VALUE (7), and FIND (33) do not exist over CANopen.
		if (propId == 6 || propId == 7 || propId == 33) {
			return false;
		}
		*index = bus::CANopen::SB_PROP_BASE + propId;
		*sub = 0;
		*nbytes = (propId == 43) ? 4 : 2;
		return true;
	}

	// Motor pucks (CiA 402 / P4 manufacturer objects).  The numeric ids
	// are the r197 (puck2 PARSE.H) property numbers: T=42, P=48,
	// MECH=66, CTS=68.  updateStatus() pins a CANopen puck's vers to
	// CANOPEN_PUCK_VERS so that property_list.cpp resolves the Property
	// enum with exactly this numbering (the raw identity revision, 1,
	// would select the pre-v33 table where T=10, P=26, MECH=28, CTS=30).
	switch (propId) {
	case 0:   *index = 0x1018; *sub = 3; *nbytes = 4; return true;  // VERS -> revision
	case 2:   *index = 0x1018; *sub = 4; *nbytes = 4; return true;  // SN
	case 8:   *index = bus::CANopen::OD_MODE_SET; *sub = 0; *nbytes = 1; return true;  // MODE
	case 42:  *index = bus::CANopen::OD_TARGET_TORQUE; *sub = 0; *nbytes = 2; return true;  // T
	case 48:  *index = bus::CANopen::OD_POSITION; *sub = 0; *nbytes = 4; return true;  // P
	case 66:  *index = 0x3012; *sub = 1; *nbytes = 2; return true;  // MECH -> RawPosition (raw mag angle 0-4095, verified on the arm as id 28 under the old table)
	case 68:  *index = bus::CANopen::OD_ENC_CONFIG; *sub = 1; *nbytes = 4; return true;  // CTS
	default:
		return false;
	}
}


Puck::Puck(const bus::CommunicationsBus& _bus, int _id) :
	bus(_bus), id(_id), vers(-1), role(-1), identityRevision(-1), type(PT_Unknown), effectiveType(PT_Unknown),
	awake(false)
{
	if ((id & NODE_ID_MASK) != id) {
		throw std::invalid_argument("Puck::Puck(): Invalid Node ID.");
	}

	updateRole();
	updateStatus();
}

Puck::~Puck()
{
}

void Puck::wake()
{
	wake(std::vector<Puck*>(1, this));
}

void Puck::saveProperty(enum Property prop) const
{
	setProperty(SAVE, getPropertyId(prop), true);
}
void Puck::saveAllProperties() const
{
	setProperty(SAVE, -1, true);
}
void Puck::resetProperty(enum Property prop) const
{
	setProperty(DEF, getPropertyId(prop));
	setProperty(LOAD, getPropertyId(prop), true);
}


void Puck::updateRole()
{
	if (id == bus::CANopen::SAFETY_NODE) {
		// The SB serves ROLE through its property map (0x2001)
		role = getProperty(ROLE);
	} else {
		// CiA-402 pucks have no BarrettCAN ROLE; they are motor drives
		// by definition of the contract's node map (section 2), with no
		// role options (no serial/optical encoders under CANopen).
		role = ROLE_TATER;
	}

	switch (role & ROLE_MASK) {
	case ROLE_SAFETY:
		type = PT_Safety;
		break;
	case ROLE_TATER:
	case ROLE_BHAND:
	case ROLE_WRAPTOR:
		type = PT_Motor;
		break;
	case ROLE_FORCE:
		type = PT_ForceTorque;
		break;
	default:
		type = PT_Unknown;
		break;
	}
}

void Puck::updateStatus()
{
	if (id == bus::CANopen::SAFETY_NODE) {
		vers = getProperty(VERS);
		int stat = getProperty(STAT);
		switch (stat) {
		case STATUS_RESET:
			effectiveType = PT_Monitor;
			break;
		case STATUS_READY:
			effectiveType = type;
			break;
		default:
			(logMessage("Puck::%s(): Bad STAT value. "
					"ID=%d, STAT=%d.")
					% __func__ % id % stat).raise<std::runtime_error>();
			break;
		}
		return;
	}

	// CiA-402 pucks: read the identity revision to prove the node is
	// up (P4 firmware boots straight to NMT pre-operational, no Monitor
	// state, so a puck that answers SDO is ready).  The revision is NOT
	// a BarrettCAN firmware version - fw 4.3.x reports 1 (BENCH-3) - so
	// it must not drive the version-keyed property tables or the
	// "old firmware" gates in the products; pin vers instead and keep
	// the raw revision for the enumeration log.
	int revision = 0;
	if (bus::CANopen::sdoUpload(bus, id, bus::CANopen::OD_IDENTITY, 3, &revision, 0.1) != 0) {
		(logMessage("Puck::%s(): ID=%d: no response reading the identity object.")
				% __func__ % id).raise<std::runtime_error>();
	}
	identityRevision = revision;
	vers = CANOPEN_PUCK_VERS;
	effectiveType = type;
}

const char Puck::puckTypeStrs[][12] = { "Monitor", "Safety", "Motor", "ForceTorque", "Unknown" };


namespace {
// NMT reset-node for every (non-NULL) puck, then collect the boot-ups.
// Restarts the pucks' position counters.
void resetAndAwaitBootUps(const bus::CommunicationsBus& bus, const std::vector<Puck*>& pucks)
{
	std::vector<Puck*>::const_iterator i;

	// Discard heartbeats buffered from the pucks' previous life, so a
	// stale frame cannot pass for the fresh boot-up (the first wake on
	// the real arm SDO-configured a puck that was still mid-reboot
	// that way).  Then reset every puck first and collect the
	// boot-ups afterwards: the resets overlap, so total bring-up time
	// is one boot (~0.4 s on a P4), not DOF boots.  The boot-ups
	// arrive in one burst, so they are awaited against one shared
	// deadline and awaitBootUp() must not drain (see canopen.h).
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		bus::CANopen::drainHeartbeats(bus, (*i)->getId());
	}
	// Stagger the resets: pucks reset back-to-back boot back-to-back,
	// and the P4 that boots last then fails to get its boot-up onto
	// the bus (opus 2026-09-22, start-1522.log: puck 8 silent after
	// every burst reset, prompt after a lone reset).  25 ms apart the
	// boot-ups land on an idle bus; the arm still wakes in well under
	// a second.
	const double RESET_STAGGER_S = 0.025;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		bus::CANopen::nmt(bus, bus::CANopen::NMT_RESET_NODE, (*i)->getId());
		btsleepRT(RESET_STAGGER_S);
	}

	const double BOOT_TIMEOUT_S = 3.0;
	const double deadline = highResolutionSystemTime() + BOOT_TIMEOUT_S;
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		const int id = (*i)->getId();
		const double remaining = deadline - highResolutionSystemTime();
		if (bus::CANopen::awaitBootUp(bus, id, remaining > 0.0 ? remaining : 0.0) == 0) {
			continue;
		}

		// One more chance for this puck alone.  The whole burst of
		// boot-ups fits in ~15 ms; a host adapter in a poor CAN error
		// state can miss one frame of it (opus 2026-09-22: node 7's
		// boot-up never reached the host while the PCAN-USB sat in
		// ERROR-PASSIVE).  A second miss is a bus problem, not bad luck.
		logMessage("Puck::wake(): Puck ID=%d sent no boot-up message within "
				"%.0fs of NMT reset-node; resetting it once more.", true)
				% id % BOOT_TIMEOUT_S;
		bus::CANopen::drainHeartbeats(bus, id);
		bus::CANopen::nmt(bus, bus::CANopen::NMT_RESET_NODE, id);
		if (bus::CANopen::awaitBootUp(bus, id, BOOT_TIMEOUT_S) == 0) {
			continue;
		}

		bool answersSdo = false;
		try {
			int value = 0;
			answersSdo = (bus::CANopen::sdoUpload(bus, id, 0x1000, 0, &value, 0.5) == 0);
		} catch (const std::runtime_error&) {
			answersSdo = true;  // an SDO abort is still an answer
		}
		(logMessage("Puck::wake(): Puck ID=%d sent no boot-up message after "
				"two NMT reset-node commands (%s). Check the host CAN "
				"interface state and error counters (ip -details -statistics "
				"link show <can>), bus termination, and puck power.")
				% id
				% (answersSdo
						? "it answers SDO requests, so its boot-up frames are being lost on the way to the host"
						: "it does not answer SDO requests either"))
				.raise<std::runtime_error>();
	}
	// Give the last-booted puck's SDO server a moment to come up
	// before the configuration writes start.
	btsleepRT(0.3);
}
}  // namespace


void Puck::wake(std::vector<Puck*> pucks, bool resetFirst)
{
	// CANopen bring-up (contract section 10, wake row): per motor puck,
	// NMT reset-node (NEVER broadcast - the SB honors a broadcast reset
	// and would cut bus power), await the boot-up message, apply the
	// section 5.3 configuration, then NMT-start the bus.  The PDO links
	// live in puck RAM and are lost on power-down, so the configuration
	// is rewritten on every bring-up; without the reset it is simply
	// rewritten over itself.  Pucks already awake are skipped, as the
	// classic wake() skipped pucks that were out of Monitor: the product
	// constructors (MultiPuckProduct) call this after
	// ProductManager::wakeAllPucks(), and a second reset would restart
	// the position counters behind the zeroing.
	std::vector<Puck*>::iterator i;
	const bus::CommunicationsBus* busPtr = NULL;

	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL  ||  (*i)->getType() != PT_Motor  ||  (*i)->isAwake()) {
			*i = NULL;  // only motor pucks not yet up take part
			continue;
		}
		busPtr = &(*i)->getBus();
	}
	if (busPtr == NULL) {
		return;
	}
	const bus::CommunicationsBus& bus = *busPtr;

	{
		// Startup resets/reconfigures the drives and must exclude cyclic
		// traffic. Take the transaction lock first to preserve lock order.
		boost::lock_guard<thread::Mutex> transactionLock(bus.getSdoMutex());
		BARRETT_SCOPED_LOCK(bus.getMutex());

		// Without a reset each drive keeps the CST mode setting written by
		// the run that armed it, so none needs enabling here - and none may
		// be: an enabled CST drive acts on whatever torque target it last
		// latched.  A drive that lost its arming, or is enabled, sends the
		// whole set down the reset path, where armCstMode() enables in the
		// torque-less mode 0 a reset leaves behind.
		for (i = pucks.begin(); !resetFirst  &&  i != pucks.end(); ++i) {
			std::string why;
			if (*i != NULL  &&  !bus::CANopen::isSensorOnly(bus, (*i)->getId())
					&&  !bus::CANopen::cstArmedAndDisabled(bus, (*i)->getId(), &why)) {
				logMessage("Puck::wake(): Puck ID=%d: %s; resetting the pucks.", true)
						% (*i)->getId() % why;
				resetFirst = true;
			}
		}
		if (resetFirst) {
			resetAndAwaitBootUps(bus, pucks);
		}

		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
				continue;
			}
			// Reconfigure with the node parked in pre-operational: no
			// TPDO can fire off a stray SYNC while its mapping is in
			// flux (a SYNC against a half-valid mapping hard-crashes
			// the P4 app - bench 2026-08-18).  The pucks self-start
			// (0x1F80 = 0), so they come out of reset Operational.
			bus::CANopen::nmt(bus, bus::CANopen::NMT_PREOP, (*i)->getId());
			if (bus::CANopen::isSensorOnly(bus, (*i)->getId())) {
				bus::CANopen::configureSensorPuck(bus, (*i)->getId());
			} else {
				bus::CANopen::configureMotorPuck(bus, (*i)->getId());
			}
		}

		// Start the bus (broadcast start is safe; the SB going NMT
		// Operational enables its status TPDO, which is desired) and
		// give the SB's IDLE-state host-liveness consumer a heartbeat.
		bus::CANopen::nmt(bus, bus::CANopen::NMT_START, 0);
		bus::CANopen::hostHeartbeat(bus);

		// Arm CST on each drive (requires motor-bus voltage, so the
		// pendant must already be idled; see armCstMode()).  Sensor-only
		// nodes have no drive to arm; warm drives are still armed.
		for (i = pucks.begin(); resetFirst  &&  i != pucks.end(); ++i) {
			if (*i == NULL  ||  bus::CANopen::isSensorOnly(bus, (*i)->getId())) {
				continue;
			}
			bus::CANopen::armCstMode(bus, (*i)->getId());
		}
	}

	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		(*i)->updateStatus();
		(*i)->awake = true;
	}
}


int Puck::StandardParser::parse(int id,
		int propId, result_type* result, const unsigned char* data, size_t len)
{
	// CANopen: an expedited SDO upload response (or an abort).
	int index, sub, nbytes;
	if (!propToOd(id, propId, &index, &sub, &nbytes)) {
		logMessage("%s: property %d has no CANopen object on node %d")
				% __func__ % propId % id;
		return 1;
	}

	if (len != 8) {
		logMessage("%s: expected an 8-byte SDO response, got length %d")
				% __func__ % len;
		return 1;
	}
	if (data[1] != (index & 0xFF)  ||  data[2] != ((index >> 8) & 0xFF)
			||  data[3] != sub) {
		logMessage("%s: SDO multiplexer mismatch: got %02x%02x:%02x, expected %04x:%02x")
				% __func__ % (int)data[2] % (int)data[1] % (int)data[3] % index % sub;
		return 1;
	}
	uint32_t abortCode = 0;
	switch (bus::CANopen::decodeUploadReply(data, result, &abortCode)) {
	case 0:
		return 0;  // (all r197 properties are signed: sign-extended)
	case 1:
		logMessage("%s: SDO abort 0x%08x reading 0x%04x:%02x on node %d")
				% __func__ % abortCode % index % sub % id;
		return 1;
	default:
		logMessage("%s: unexpected/non-expedited SDO response 0x%02x")
				% __func__ % (int)data[0];
		return 1;
	}
}


}

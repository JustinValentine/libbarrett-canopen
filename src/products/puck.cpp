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
		// Every property p at 0x2000 + p, except ADDR (6), VALUE (7) and
		// FIND (33).  PEN (43) is the only 32-bit one.
		if (propId == 6 || propId == 7 || propId == 33) {
			return false;
		}
		*index = bus::CANopen::SB_PROP_BASE + propId;
		*sub = 0;
		*nbytes = (propId == 43) ? 4 : 2;
		return true;
	}

	// Motor pucks: CiA-402 and P4 manufacturer objects, keyed by the r197
	// property numbers (see CANOPEN_PUCK_VERS).
	switch (propId) {
	case 0:   *index = 0x1018; *sub = 3; *nbytes = 4; return true;  // VERS -> revision
	case 2:   *index = 0x1018; *sub = 4; *nbytes = 4; return true;  // SN
	case 8:   *index = bus::CANopen::OD_MODE_SET; *sub = 0; *nbytes = 1; return true;  // MODE
	case 42:  *index = bus::CANopen::OD_TARGET_TORQUE; *sub = 0; *nbytes = 2; return true;  // T
	case 48:  *index = bus::CANopen::OD_POSITION; *sub = 0; *nbytes = 4; return true;  // P
	case 66:  *index = bus::CANopen::OD_RAW_POSITION; *sub = 1; *nbytes = 2; return true;  // MECH -> single-turn encoder, 0-4095
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
		role = getProperty(ROLE);
	} else {
		// P4 pucks have no ROLE property: nodes 1-8 are motor drives
		// (contract section 2), without role options.
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

	// P4 pucks have no Monitor state: one that answers SDO is ready.  Its
	// identity revision is not a BarrettCAN firmware version, so vers is
	// fixed instead (see CANOPEN_PUCK_VERS).
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
// NMT reset-node for every (non-NULL) puck, then wait for the boot-ups.
// Restarts the pucks' position counters.
void resetAndAwaitBootUps(const bus::CommunicationsBus& bus, const std::vector<Puck*>& pucks)
{
	std::vector<Puck*>::const_iterator i;

	// Discard heartbeats buffered before the reset, so that an old frame
	// cannot pass for the new boot-up.  Then reset all the pucks and wait
	// for their boot-ups against one shared deadline.
	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL) {
			continue;
		}
		bus::CANopen::drainHeartbeats(bus, (*i)->getId());
	}
	// Stagger the resets: of eight P4s reset back to back, the last to
	// boot can fail to get its boot-up message onto the bus.
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

		// One more try for this puck alone: a host adapter in a poor CAN
		// error state can miss one frame of the burst.
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
	// Give the last puck's SDO server time to come up
	btsleepRT(0.3);
}
}  // namespace


void Puck::wake(std::vector<Puck*> pucks, bool resetFirst)
{
	// Bring-up (contract section 10): reset each motor puck (never by
	// broadcast), wait for its boot-up, write the section 5.3
	// configuration and start the bus.  The PDO configuration is not
	// saved on the pucks, so it is written on every bring-up.  Pucks that
	// are already awake are skipped: the product constructors call this
	// again, and a second reset would restart the position counters after
	// the WAM was zeroed.
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
		// Exclude both SDO users and the cyclic traffic (SDO mutex first)
		boost::lock_guard<thread::Mutex> transactionLock(bus.getSdoMutex());
		BARRETT_SCOPED_LOCK(bus.getMutex());

		// Skipping the reset requires every drive to still be in CST mode
		// and disabled; armCstMode() must only enable a drive that has
		// just been reset.  Otherwise reset them all.
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

		// P4 firmware 4.3.2 keeps its saved PDO transmission types through
		// the first configuration after a reset, although every object
		// reads back as written.  A puck saved with TPDO2 type 0 would then
		// send its statusword on every SYNC.  So after a reset the
		// configuration is written twice.
		const int setupPasses = resetFirst ? 2 : 1;
		for (int pass = 0; pass < setupPasses; ++pass) {
			for (i = pucks.begin(); i != pucks.end(); ++i) {
				if (*i == NULL) {
					continue;
				}
				// Configure in pre-operational (see configureMotorPuck());
				// the pucks come out of reset Operational.
				bus::CANopen::nmt(bus, bus::CANopen::NMT_PREOP, (*i)->getId());
				if (bus::CANopen::isSensorOnly(bus, (*i)->getId())) {
					bus::CANopen::configureSensorPuck(bus, (*i)->getId());
				} else {
					bus::CANopen::configureMotorPuck(bus, (*i)->getId());
				}
			}
		}

		// Start every node, the safety board included
		bus::CANopen::nmt(bus, bus::CANopen::NMT_START, 0);
		bus::CANopen::hostHeartbeat(bus);

		// Select CST mode on each drive that was reset
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
	// An expedited SDO upload response, or an abort
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
		return 0;
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

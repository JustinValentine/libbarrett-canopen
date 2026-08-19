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
	// are the stable r197 property numbers that property_list resolves
	// to for modern firmware.
	switch (propId) {
	case 0:   *index = 0x1018; *sub = 3; *nbytes = 4; return true;  // VERS -> revision
	case 2:   *index = 0x1018; *sub = 4; *nbytes = 4; return true;  // SN
	case 8:   *index = bus::CANopen::OD_MODE_SET; *sub = 0; *nbytes = 1; return true;  // MODE
	case 42:  *index = bus::CANopen::OD_TARGET_TORQUE; *sub = 0; *nbytes = 2; return true;  // T
	case 48:  *index = bus::CANopen::OD_POSITION; *sub = 0; *nbytes = 4; return true;  // P
	case 68:  *index = bus::CANopen::OD_ENC_CONFIG; *sub = 1; *nbytes = 4; return true;  // CTS
	default:
		return false;
	}
}


Puck::Puck(const bus::CommunicationsBus& _bus, int _id) :
	bus(_bus), id(_id), vers(-1), role(-1), type(PT_Unknown), effectiveType(PT_Unknown)
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

	// CiA-402 pucks: firmware revision from the identity object.  P4
	// firmware boots straight to NMT pre-operational (no Monitor
	// state), so a puck that answers SDO is ready.
	if (bus::CANopen::sdoUpload(bus, id, bus::CANopen::OD_IDENTITY, 3, &vers, 0.1) != 0) {
		(logMessage("Puck::%s(): ID=%d: no response reading the identity object.")
				% __func__ % id).raise<std::runtime_error>();
	}
	effectiveType = type;
}

const char Puck::puckTypeStrs[][12] = { "Monitor", "Safety", "Motor", "ForceTorque", "Unknown" };


void Puck::wake(std::vector<Puck*> pucks)
{
	// CANopen bring-up (contract section 10, wake row): per motor puck,
	// NMT reset-node (NEVER broadcast - the SB honors a broadcast reset
	// and would cut bus power), await the boot-up message, apply the
	// section 5.3 configuration, then NMT-start the bus.  Re-running
	// the configuration is idempotent, so this happens on every product
	// construction; the PDO links live in puck RAM and are lost on
	// power-down.
	std::vector<Puck*>::iterator i;
	const bus::CommunicationsBus* busPtr = NULL;

	for (i = pucks.begin(); i != pucks.end(); ++i) {
		if (*i == NULL  ||  (*i)->getType() != PT_Motor) {
			*i = NULL;  // only motor pucks take part in the bring-up
			continue;
		}
		busPtr = &(*i)->getBus();
	}
	if (busPtr == NULL) {
		return;
	}
	const bus::CommunicationsBus& bus = *busPtr;

	{
		// Serialize the whole bring-up against other bus users
		BARRETT_SCOPED_LOCK(bus.getMutex());

		// Reset every puck first, then collect the boot-ups: the resets
		// overlap, so total bring-up time is one boot, not DOF boots.
		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
				continue;
			}
			bus::CANopen::nmt(bus, bus::CANopen::NMT_RESET_NODE, (*i)->getId());
		}

		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
				continue;
			}
			if (bus::CANopen::awaitBootUp(bus, (*i)->getId(), 3.0) != 0) {
				(logMessage("Puck::%s(): Puck ID=%d did not send a boot-up "
						"message within 3s of NMT reset-node.")
						% __func__ % (*i)->getId()).raise<std::runtime_error>();
			}
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
			bus::CANopen::configureMotorPuck(bus, (*i)->getId());
		}

		// Start the bus (broadcast start is safe; the SB going NMT
		// Operational enables its status TPDO, which is desired) and
		// give the SB's IDLE-state host-liveness consumer a heartbeat.
		bus::CANopen::nmt(bus, bus::CANopen::NMT_START, 0);
		bus::CANopen::hostHeartbeat(bus);

		// Arm CST on each drive (requires motor-bus voltage, so the
		// pendant must already be idled; see armCstMode()).
		for (i = pucks.begin(); i != pucks.end(); ++i) {
			if (*i == NULL) {
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
	if (data[0] == 0x80) {  // SDO abort
		int abortCode = data[4] | ((int)data[5] << 8)
				| ((int)data[6] << 16) | ((int)data[7] << 24);
		logMessage("%s: SDO abort 0x%08x reading 0x%04x:%02x on node %d")
				% __func__ % abortCode % index % sub % id;
		return 1;
	}
	if ((data[0] & 0xE0) != 0x40  ||  !(data[0] & 0x02)) {
		logMessage("%s: unexpected/non-expedited SDO response 0x%02x")
				% __func__ % (int)data[0];
		return 1;
	}

	// Transferred size, then sign-extend (all r197 properties are signed)
	int n = (data[0] & 0x01) ? 4 - ((data[0] >> 2) & 3) : 4;
	*result = (data[3 + n] & 0x80) ? -1 : 0;
	for (int i = 3 + n; i >= 4; --i) {
		*result = (*result << 8) | data[i];
	}
	return 0;
}


}

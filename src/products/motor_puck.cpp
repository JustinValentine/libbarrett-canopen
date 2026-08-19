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
 * motor_puck.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: cd
 *      Author: dc
 */

#include <stdexcept>
#include <limits>

#include <boost/tuple/tuple.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/canopen.h>
#include <barrett/math/utils.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>


namespace barrett {


void MotorPuck::setPuck(Puck* puck)
{
	// Call super
	SpecialPuck::setPuck(puck);

	if (p != NULL) {
		// CANopen: counts per revolution from the encoder configuration
		// object, torque scale from the motor rated torque (contract
		// section 5.2).  IPNM is a BarrettCAN concept and stays unset.
		cts = 0;
		if (bus::CANopen::sdoUpload(p->getBus(), p->getId(),
				bus::CANopen::OD_ENC_CONFIG, 1, &cts) != 0  ||  cts <= 0) {
			(logMessage("MotorPuck::%s(): ID=%d: bad encoder resolution %d.")
					% __func__ % p->getId() % cts).raise<std::runtime_error>();
		}
		rpc = 2*M_PI / cts;
		cpr = cts / (2*M_PI);

		int ratedTorque_mNm = bus::CANopen::readRatedTorque(p->getBus(), p->getId());
		ppnm = 1.0e6 / ratedTorque_mNm;  // (1000 permille/rated) * (1000 mNm/Nm)
		ipnm = 0;
	}
}


void MotorPuck::sendPackedTorques(const bus::CommunicationsBus& bus, int groupId, int propId,
		const double* pt, int numTorques)
{
	(void)propId;  // CANopen frames carry no property byte

	unsigned char data[8];

	if (numTorques < 0  ||  numTorques > 4) {
		throw std::logic_error("MotorPuck::sendPackedTorques(): numTorques must be >= 0 and <= PUCKS_PER_TORQUE_GROUP.");
	}

	// Torque quad (contract sections 6.1/6.2): four i16 little-endian
	// per-mille-of-rated-torque slots.  Unused slots are zero, which is
	// also the correct pad for quad B's reserved bytes 6-7.
	for (int i = 0; i < 4; ++i) {
		int permille = (i >= numTorques) ? 0
				: (int)floor(math::saturate(pt[i], (double)MAX_TORQUE_PERMILLE));
		data[2*i]     = permille & 0xFF;
		data[2*i + 1] = (permille >> 8) & 0xFF;
	}

	bus.send(groupId, data, 8);  // groupId is the quad COB (0x201/0x205)
}


}

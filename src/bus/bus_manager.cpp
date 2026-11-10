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
 * bus_manager.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/bus_manager.h>


namespace barrett {
namespace bus {


BusManager::BusManager(CommunicationsBus* _bus) :
	bus(_bus), deleteBus(false), messageBuffers()
{
	if (bus == NULL) {
		bus = new CANSocket;
		deleteBus = true;
	}
}

BusManager::BusManager(int port) :
	bus(NULL), deleteBus(true), messageBuffers()
{
	bus = new CANSocket(port);
}

BusManager::~BusManager()
{
	if (deleteBus) {
		delete bus;
	}
}

int BusManager::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking, bool realtime) const
{
	const double start = highResolutionSystemTime();
	while (true) {
		{
			// Protect buffers and one nonblocking receive pass only. SDO
			// ownership is protected separately by getSdoMutex(): a lost
			// status reply must not stop SYNC/PDO/heartbeat traffic.
			BARRETT_SCOPED_LOCK(getMutex());
			if (retrieveMessage(expectedBusId, data, len)) return 0;
			const int ret = updateBuffers();
			if (ret != 0) return ret;
			if (retrieveMessage(expectedBusId, data, len)) return 0;
			if (!blocking) return 1;
		}

		double now = highResolutionSystemTime();
		if ((now - start) > CommunicationsBus::TIMEOUT) {
			logMessage("BusManager::receive(): timed out. Now: %lf, Start: %lf", true) %now %start;
			return 2;
		}

		if (!realtime) btsleepRT(0.0001);
	}
}

int BusManager::updateBuffers() const
{
	BARRETT_SCOPED_LOCK(getMutex());

	int busId;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;
	int ret;

	// empty the bus' receive buffer
	while (true) {
		ret = receiveRaw(busId, data, len, false);  // non-blocking read
		if (ret == 0) {  // successfully received a message
			// Buffers are keyed by raw CAN identifier, which under
			// CANopen is the COB-ID.  (The old BarrettCAN special case
			// that dropped busId 0x540 is gone.)
			storeMessage(busId, data, len);
		} else if (ret == 1) {  // would block
			return 0;
		} else {  // error
			return ret;
		}
	}
}

void BusManager::storeMessage(int busId, const unsigned char* data, size_t len) const
{
	// CANopen carries continuous unsolicited traffic the host does not
	// always consume (statusword TPDOs, heartbeats, EMCYs, the SB
	// status TPDO).  The buffers are rings: when one is full, push_back
	// overwrites the oldest entry - newest-wins is the correct policy
	// for cyclic telemetry, and consumers of request/reply traffic
	// (SDO) drain their buffers promptly.  (The old code raised on
	// overflow, which would kill the control thread within seconds of
	// the first unconsumed periodic broadcast.)
	messageBuffers[busId].push_back(Message(data, len));
}

bool BusManager::retrieveMessage(int busId, unsigned char* data, size_t& len) const
{
	if (messageBuffers[busId].empty()) {
		return false;
	}

	messageBuffers[busId].front().copyTo(data, len);
	messageBuffers[busId].pop_front();

	return true;
}


}
}

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
 * @file real_time_execution_manager.cpp
 * @date 12/11/2009
 * @author Dan Cody
 * 
 */

#include <stdexcept>
#include <string>
#include <limits>
#include <cmath>
#include <cassert>
#include <algorithm>

#include <errno.h>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <barrett/os.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/thread/disable_secondary_mode_warning.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/real_time_execution_manager.h>


namespace barrett {
namespace systems {


// TODO(dc): test!


RealTimeExecutionManager::RealTimeExecutionManager(double period_s, int rt_priority) :
	ExecutionManager(period_s),
	thread(), priority(rt_priority), running(false), error(false), errorStr(), errorCallback()
{
	init();
}

RealTimeExecutionManager::RealTimeExecutionManager(const libconfig::Setting& setting) :
	ExecutionManager(setting),
	thread(), priority(), running(false), error(false), errorStr(), errorCallback()
{
	priority = setting["thread_priority"];
	init();
}


RealTimeExecutionManager::~RealTimeExecutionManager()
{
	stop();
}


void RealTimeExecutionManager::start()
{
	BARRETT_SCOPED_LOCK(getMutex());

	if (getError()) {
		throw std::logic_error("systems::RealTimeExecutionManager::start(): Cannot start while in an error state. Call RealTimeExecutionManager::clearError() first.");
	}

	if ( !isRunning() ) {
		thread::DisableSecondaryModeWarning dsmw;

		if (thread.joinable()) thread.join();
		std::shared_ptr<std::promise<void> > startup(new std::promise<void>);
		std::future<void> ready = startup->get_future();
		boost::thread tmpThread(&RealTimeExecutionManager::executionLoopEntryPoint, this, startup);
		thread.swap(tmpThread);

		// Setup failures must reach the caller, not escape a worker and
		// terminate the process (or leave start() spinning indefinitely).
		try {
			ready.get();
		} catch (const std::exception& e) {
			thread.join();
			errorStr = e.what();
			error = true;
			throw;
		} catch (...) {
			thread.join();
			errorStr = "Unknown control-thread startup failure";
			error = true;
			throw;
		}
	}
	// TODO(dc): else, throw an exception?
}

void RealTimeExecutionManager::stop()
{
	if (thread.joinable()) {
		thread.interrupt();
		thread.join();
	}
	assert( !isRunning() );
}

void RealTimeExecutionManager::clearError()
{
	// Do not join while holding the mutex the worker uses to report errors.
	stop();
	BARRETT_SCOPED_LOCK(getMutex());

	error = false;
	errorStr = "";
}

void RealTimeExecutionManager::setErrorCallback(callback_type callback)
{
	BARRETT_SCOPED_LOCK(getMutex());

	errorCallback = callback;
}

void RealTimeExecutionManager::clearErrorCallback()
{
	setErrorCallback(callback_type());
}

void RealTimeExecutionManager::executionLoopEntryPoint(std::shared_ptr<std::promise<void> > startup)
{
	std::unique_ptr<PeriodicLoopTimer> loopTimer;
	try {
		barrett::detail::configureRealtimeThread(priority);
		loopTimer.reset(new PeriodicLoopTimer(period, priority));
	} catch (...) {
		startup->set_exception(std::current_exception());
		return;
	}
	running = true;
	startup->set_value();
	startup.reset();

	uint32_t period_us = period * 1e6;
	double start;
	uint32_t duration;
	uint32_t min = std::numeric_limits<unsigned int>::max();
	uint32_t max = 0;
	uint64_t sum = 0;
	uint64_t sumSq = 0;
	uint32_t loopCount = 0;
	uint32_t overruns = 0;
	uint32_t missedReleasePoints = 0;

	try {
		while (true) {
			// Explicit interruption point
			boost::this_thread::interruption_point();

			missedReleasePoints += loopTimer->wait();
			start = highResolutionSystemTime();

			runExecutionCycle();

            duration = (highResolutionSystemTime() - start) * 1e6;
			if (duration < min) {
				min = duration;
			}
			if (duration > max) {
				max = duration;
			}
			sum += duration;
			sumSq += static_cast<uint64_t>(duration) * duration;
			++loopCount;
			if (duration > period_us) {
				++overruns;
			}
		}
	} catch (const boost::thread_interrupted& e) {
		// Interruption requested, probably by stop(). Do nothing.
	} catch (const std::exception& e) {
		// Say so: the thread ends here, and a program idling in
		// waitForMode() would otherwise never learn why the WAM went quiet.
		logMessage("RealTimeExecutionManager: control loop stopped: %s", true) % e.what();
		BARRETT_SCOPED_LOCK(getMutex());

		errorStr = e.what();
		error = true;

		if (errorCallback) {
			errorCallback(this, ExecutionManagerException(e.what()));
		}
	}
	running = false;


	double mean = loopCount ? (double)sum / loopCount : 0.0;
	double stdev = loopCount ? std::sqrt(std::max(0.0, ((double)sumSq/loopCount) - mean*mean)) : 0.0;
	if (!loopCount) min = 0;

    logMessage("RealTimeExecutionManager control-loop stats (microseconds):");
    logMessage("  target period = %u") % period_us;
    logMessage("  min = %u") % min;
    logMessage("  ave = %.3f") % mean;
    logMessage("  max = %u") % max;
    logMessage("  stdev = %.3f") % stdev;
    logMessage("  num total cycles = %u") % loopCount;
    logMessage("  num missed release points = %u") % missedReleasePoints;
    logMessage("  num overruns = %u") % overruns;
}

void RealTimeExecutionManager::init()
{
	// install a more appropriate mutex
	delete mutex;
	mutex = new thread::RealTimeMutex;  // ~ExecutionManager() will delete this

	//errorCallback = boost::bind(std::terminate);
}




}
}

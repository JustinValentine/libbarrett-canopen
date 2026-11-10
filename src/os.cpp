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
 */

/**
 * @file os.cpp
 * @date 03/28/2012
 * @author Dan Cody
 * 
 */


#include <stdexcept>
#include <iostream>
#include <cassert>
#include <cmath>
#include <limits>
#include <system_error>
#include <cerrno>

#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

#ifdef BARRETT_XENOMAI
#include <native/task.h>
#include <native/timer.h>
#endif

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

#include <barrett/detail/stacktrace.h>
#include <barrett/detail/os.h>
#include <barrett/os.h>


#ifdef BARRETT_XENOMAI
// Xenomai helper function
inline RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}

// Xenomai requires at least one call to mlockall() per process. Also, a signal
// handler is installed to trap transitions from primary execution mode to
// secondary execution mode; this aids in identifying code that breaks Xenomai's
// realtime guarantee.
namespace {  // Using an anonymous namespace because no other code needs to
			 // interact with these declarations. It is necessary to construct a
			 // single instance of InitXenomai.
	extern "C" {
	void warnOnSwitchToSecondaryMode(int)
	{
		barrett::logMessage("WARNING: Switched out of RealTime. Stack-trace:", true);
		barrett::detail::syslog_stacktrace();
	}
	}

	class InitXenomai {
	public:
		InitXenomai() {
			// Avoids memory swapping for this program
			mlockall(MCL_CURRENT|MCL_FUTURE);

			// Handler for warnings about falling out of primary mode
			signal(SIGXCPU, &warnOnSwitchToSecondaryMode);
		}
	};
	// Static variables are initialized when the module is loaded. This causes the
	// InitXenomai::InitXenomai() ctor to be called at module load time.
	static InitXenomai ignore;
}
#endif


namespace barrett {


void btsleep(double duration_s)
{
	// Why do we need duration_s to be > 1 us? 
	// This would call abort() for durations under 1 us, which seems bad!
	//assert(duration_s > 1e-6);  // Minimum duration is 1 us
	
	boost::this_thread::sleep(boost::posix_time::microseconds(long(duration_s * 1e6)));
}

void btsleepRT(double duration_s)
{
#ifdef BARRETT_XENOMAI
	assert(duration_s > 1e-9);  // Minimum duration is 1 ns
	int ret = rt_task_sleep(RTIME(duration_s * 1e9));
	if (ret != 0) {
		(logMessage("%s: rt_task_sleep() returned error %d.") % __func__ % ret).raise<std::runtime_error>();
	}
#else
	btsleep(duration_s);
#endif
}

void btsleep(double duration_s, bool realtime)
{
	if (realtime) {
		btsleepRT(duration_s);
	} else {
		btsleep(duration_s);
	}
}

double highResolutionSystemTime()
{
#ifdef BARRETT_XENOMAI
	return 1e-9 * rt_timer_read();
#else
	struct timespec now;
	if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
		throw std::system_error(errno, std::generic_category(), "clock_gettime(CLOCK_MONOTONIC)");
	}
	return static_cast<double>(now.tv_sec) + now.tv_nsec * 1e-9;
#endif
}

void detail::configureRealtimeThread(int priority)
{
#if defined(__linux__) && !defined(BARRETT_XENOMAI)
	if (priority == 0) return;  // Explicit offline/non-RT mode, never a fallback.
	if (priority < sched_get_priority_min(SCHED_FIFO) ||
			priority > sched_get_priority_max(SCHED_FIFO)) {
		throw std::invalid_argument("Real-time thread priority must be 1..99 (or 0 for offline tests).");
	}
	struct sched_param param = {};
	param.sched_priority = priority;
	const int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
	if (ret != 0) {
		throw std::system_error(ret, std::generic_category(),
				"Cannot start control thread with SCHED_FIFO; run with RT permissions (e.g. sudo)");
	}
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
		throw std::system_error(errno, std::generic_category(),
				"Cannot lock control-process memory; check CAP_IPC_LOCK / RLIMIT_MEMLOCK");
	}
	// Touch enough stack for the usual control path before it starts executing.
	// This is not a substitute for avoiding allocations inside operate().
	volatile unsigned char stack[64 * 1024];
	for (size_t i = 0; i < sizeof(stack); ++i) stack[i] = 0;
	logMessage("Control thread: SCHED_FIFO priority %d; memory locked") % priority;
#else
	(void)priority;
#endif
}

#ifndef BARRETT_XENOMAI
// period in us
static int make_periodic (unsigned int period, struct periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create (CLOCK_MONOTONIC, TFD_CLOEXEC);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period/1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime (fd, 0, &itval, NULL);
	if (ret != 0) {
		const int savedErrno = errno;
		close(fd);
		info->timer_fd = -1;
		errno = savedErrno;
	}
	return ret;
}

static void wait_period (struct periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	do {
		ret = read(info->timer_fd, &missed, sizeof(missed));
	} while (ret == -1 && errno == EINTR);
	if (ret != sizeof(missed)) {
		throw std::system_error(ret == -1 ? errno : EIO,
				std::generic_category(), "read(timerfd)");
	}

	/* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
	info->wakeups_missed = missed > 0 ? missed - 1 : 0;
}
#endif

PeriodicLoopTimer::PeriodicLoopTimer(double period_, int threadPriority) :
		firstRun(true), period(period_), releasePoint(-1.0)
{
	if (!std::isfinite(period) || period < 1e-6 ||
			period * 1e6 > std::numeric_limits<unsigned int>::max()) {
		throw std::invalid_argument("PeriodicLoopTimer requires a finite period of at least 1 microsecond.");
	}
#ifdef BARRETT_XENOMAI
	int ret;

	// Try to become a Xenomai task
	ret = rt_task_shadow(NULL, NULL, threadPriority, 0);
	// EBUSY indicates the current thread is already a Xenomai task
	if (ret != 0  &&  ret != -EBUSY) {
		(logMessage("PeriodicLoopTimer::%s: rt_task_shadow(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	ret = rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(period));
	if (ret != 0) {
		(logMessage("PeriodicLoopTimer::%s: rt_task_set_periodic(): (%d) %s")
				% __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}
#else
	(void)threadPriority;  // The execution manager configures its Linux worker.
	if (make_periodic(period * 1e6, &info) != 0) {
		throw std::system_error(errno, std::generic_category(), "create/arm periodic timerfd");
	}
#endif
}

PeriodicLoopTimer::~PeriodicLoopTimer()
{
#ifndef BARRETT_XENOMAI
	close(info.timer_fd);
#endif
}

unsigned long PeriodicLoopTimer::wait()
{
#ifdef BARRETT_XENOMAI
	unsigned long missedReleasePoints;

	int ret = rt_task_wait_period(&missedReleasePoints);
	if (ret != 0  &&  ret != -ETIMEDOUT) {  // ETIMEDOUT means that we missed a release point
		(logMessage("%s: rt_task_wait_period(): (%d) %s") % __func__ % -ret % strerror(-ret)).raise<std::runtime_error>();
	}

	return missedReleasePoints;
#else
	wait_period(&info);
	return info.wakeups_missed;
/*
	const double now = highResolutionSystemTime();	// Get the current time
	const double remainder = releasePoint - now; 	// Calculate the amount of time remaining until our next release point
	if (remainder <= 0) { 							// If we are already past our next scheduled release point, then we missed it
		releasePoint = now + period; 				// Schedule a new release point at period seconds into the future

		if (firstRun) {
			firstRun = false;
			return 0;								// Don't worry if we missed the very first release point
		} else {									// Otherwise, report the number of periods missed
			return ceil(-remainder / period);		// ceil() returns the smallest integer not less than arg
		}
	} else {
		// Calculate the new releasePoint based on the old one.
		// This eliminates drift (on average) due to over/under sleeping.
		releasePoint += period;
		btsleep(remainder);
		return 0;
	}
*/
#endif
}



detail::LogFormatter logMessage(const std::string& message, bool outputToStderr)
{
	return detail::LogFormatter(message, outputToStderr);
}


namespace detail {

void LogFormatter::print()
{
	// Make sure we only print once
	if (printed) {
		return;
	}
	printed = true;

	std::string message = str();
	if (ose) {
		std::cerr << message;
		// The message should always have one newline
		if (message[message.size() - 1] != '\n') {
			std::cerr << std::endl;
		}
	}
	// Use a trivial format string in case message contains '%'
	syslog(LOG_ERR, "%s", message.c_str());
}

}
}

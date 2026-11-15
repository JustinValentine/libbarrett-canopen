// Exercise production Linux scheduling/timers/EM/mutexes without opening CAN.
// Only privileged calls and injected failures are wrapped; timers and locks
// otherwise use the real kernel. --probe uses real privilege calls as well.
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <dirent.h>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <system_error>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <barrett/os.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/real_time_execution_manager.h>

namespace {
std::atomic<int> schedulerError(0), memoryError(0), createError(0), armError(0), readError(0);
std::atomic<int> schedulerCalls(0), memoryCalls(0), piCalls(0);
pthread_t parentThread, workerThread;
bool probe = false;

int fdCount() {
	DIR* dir = opendir("/proc/self/fd");
	assert(dir);
	int count = 0;
	while (readdir(dir)) ++count;
	closedir(dir);
	return count;
}

template<typename F> void mustThrow(F f) {
	bool caught = false;
	try { f(); } catch (const std::exception&) { caught = true; }
	assert(caught);
}

void waitForStop(barrett::systems::RealTimeExecutionManager& em) {
	for (int i = 0; i < 1000 && em.isRunning(); ++i)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	assert(!em.isRunning());
}
}

extern "C" {
int __real_pthread_setschedparam(pthread_t, int, const struct sched_param*);
int __wrap_pthread_setschedparam(pthread_t thread, int policy, const struct sched_param* param) {
	++schedulerCalls;
	assert(!pthread_equal(thread, parentThread));
	assert(policy == SCHED_FIFO && param->sched_priority == 50);
	workerThread = thread;
	return probe ? __real_pthread_setschedparam(thread, policy, param) : schedulerError.load();
}
int __real_mlockall(int);
int __wrap_mlockall(int flags) {
	++memoryCalls;
	assert(flags == (MCL_CURRENT | MCL_FUTURE));
	if (probe) return __real_mlockall(flags);
	if (memoryError) { errno = memoryError; return -1; }
	return 0;
}
int __real_pthread_mutexattr_setprotocol(pthread_mutexattr_t*, int);
int __wrap_pthread_mutexattr_setprotocol(pthread_mutexattr_t* attr, int protocol) {
	assert(protocol == PTHREAD_PRIO_INHERIT);
	++piCalls;
	return __real_pthread_mutexattr_setprotocol(attr, protocol);
}
int __real_timerfd_create(int, int);
int __wrap_timerfd_create(int clock, int flags) {
	assert(clock == CLOCK_MONOTONIC && (flags & TFD_CLOEXEC));
	if (createError) { errno = createError; return -1; }
	return __real_timerfd_create(clock, flags);
}
int __real_timerfd_settime(int, int, const struct itimerspec*, struct itimerspec*);
int __wrap_timerfd_settime(int fd, int flags, const struct itimerspec* value, struct itimerspec* old) {
	if (armError) { errno = armError; return -1; }
	return __real_timerfd_settime(fd, flags, value, old);
}
ssize_t __real_read(int, void*, size_t);
ssize_t __wrap_read(int fd, void* data, size_t len) {
	const int error = readError.exchange(0);
	if (error) { errno = error; return -1; }
	return __real_read(fd, data, len);
}
}

int main(int argc, char** argv) {
	using barrett::PeriodicLoopTimer;
	using barrett::systems::RealTimeExecutionManager;
	parentThread = pthread_self();
	probe = argc == 2 && std::string(argv[1]) == "--probe";
	if (probe) {
		try {
			RealTimeExecutionManager em(0.002, 50);
			em.start();
			int policy = -1;
			struct sched_param param = {};
			assert(pthread_getschedparam(workerThread, &policy, &param) == 0);
			assert(policy == SCHED_FIFO && param.sched_priority == 50);
			std::ifstream status("/proc/self/status");
			std::string line;
			while (std::getline(status, line))
				if (line.find("VmLck:") == 0) std::cout << line << '\n';
			em.stop();
			std::cout << "Hardware-free probe passed: control worker SCHED_FIFO/50.\n";
			return 0;
		} catch (const std::exception& e) {
			std::cerr << "Startup rejected cleanly: " << e.what() << '\n';
			return 2;
		}
	}

	// syslog may lazily open a process-wide socket; exclude that from the
	// timer-descriptor leak check.
	barrett::logMessage("Starting hardware-free Linux real-time tests");
	const int fds = fdCount();
	for (double period : {0.0, -0.001, 1e-9, std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::quiet_NaN()})
		mustThrow([&] { PeriodicLoopTimer timer(period); });
	createError = EMFILE;
	mustThrow([] { PeriodicLoopTimer timer(0.002); });
	createError = 0;
	armError = EINVAL;
	mustThrow([] { PeriodicLoopTimer timer(0.002); });
	armError = 0;
	assert(fdCount() == fds);
	{
		PeriodicLoopTimer timer(0.002);
		readError = EINTR;
		timer.wait();  // An interrupted read must retry, not count a cycle.
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		assert(timer.wait() >= 5);
		readError = EIO;
		mustThrow([&] { timer.wait(); });
	}
	assert(fdCount() == fds);
	const double before = barrett::highResolutionSystemTime();
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
	assert(barrett::highResolutionSystemTime() > before);

	{
		barrett::thread::RealTimeMutex mutex;
		mutex.lock();
		assert(mutex.try_lock());
		std::thread contender([&] { assert(!mutex.try_lock()); });
		contender.join();
		const int depth = mutex.fullUnlock();
		assert(depth == 2);
		std::thread next([&] { assert(mutex.try_lock()); mutex.unlock(); });
		next.join();
		mutex.relock(depth);
		mutex.unlock();
		mutex.unlock();
	}
	assert(piCalls > 0);

	{
		RealTimeExecutionManager invalid(0.002, 100);
		mustThrow([&] { invalid.start(); });
		assert(invalid.getError() && !invalid.isRunning());
		invalid.clearError();
	}
	{
		RealTimeExecutionManager em(0.002, 50);
		schedulerError = EPERM;
		mustThrow([&] { em.start(); });
		assert(em.getError() && !em.isRunning());
		assert(em.getErrorStr().find("SCHED_FIFO") != std::string::npos);
		assert(memoryCalls == 0);
		em.clearError();
		schedulerError = 0;
		memoryError = ENOMEM;
		mustThrow([&] { em.start(); });
		assert(em.getErrorStr().find("memory") != std::string::npos);
		em.clearError();
		memoryError = 0;
		createError = EMFILE;
		mustThrow([&] { em.start(); });
		em.clearError();
		createError = 0;
		em.start();
		assert(em.isRunning() && !em.getError());
		em.stop();
		em.stop();
		for (int i = 0; i < 100; ++i) { em.start(); em.stop(); }
	}
	assert(fdCount() == fds);
	{
		RealTimeExecutionManager em(0.002, 0);
		const int calls = schedulerCalls;
		em.start();
		em.stop();
		assert(schedulerCalls == calls);  // Explicit non-RT mode.
	}
	{
		RealTimeExecutionManager em(0.002, 50);
		std::atomic<int> callbacks(0);
		em.setErrorCallback([&](RealTimeExecutionManager*, const barrett::systems::ExecutionManagerException&) {
			++callbacks;
		});
		readError = EIO;
		em.start();
		waitForStop(em);
		assert(em.getError() && callbacks == 1);
		em.clearError();
		assert(!em.getError());
	}
	int parentPolicy = -1;
	struct sched_param parentParam = {};
	assert(pthread_getschedparam(parentThread, &parentPolicy, &parentParam) == 0);
	assert(parentPolicy == SCHED_OTHER);
	assert(fdCount() == fds);
	std::cout << "PASS: RT startup/errors, timer failures/EINTR/cleanup, mutex recursion/PI, 100 restarts, runtime-error shutdown.\n";
}

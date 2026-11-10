/* Gentle torque-sign check for the CANopen port.
 *
 * After the normal activation (arm supported, at home) the torque on ONE
 * joint is ramped up slowly from zero, everything else at zero and no
 * gravity compensation.  The ramp stops by itself the moment the joint
 * or ANY OTHER joint has moved a small amount (default 0.02 rad), or when the torque cap or
 * the time limit is reached, and torque returns to zero.  It reports the
 * direction of the motion and the torque at which it started (a rough
 * static-friction figure).  A positive torque must move the joint in the
 * positive direction.
 *
 *   ./lbc_torque_sign [joint=4] [maxNm=3.0] [rampSeconds=3.0] [stopRad=0.02] [--hold-j7]
 * Optional WAM8 J8 test: use configured J7 PD gains to hold its starting
 * angle during the short ramp. Both torques go to zero together at the stop.
 *
 * Keep Shift-Idle ready and support the other joints.
 */
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <csignal>
#include <atomic>
#include <unistd.h>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>
#include "torque_sign_ramp.h"

using namespace barrett;

static_assert(ATOMIC_INT_LOCK_FREE==2,"Signal flag must be lock-free");
static std::atomic<int> interrupted{0};
static std::atomic<int> rampActive{0};
static int joint=4;
static double maxNm=3,rampS=3,stopRad=.02;
static bool holdJ7=false;
static void signalHandler(int signal) {
    if(!rampActive.load(std::memory_order_relaxed)) {
        // Nothing is commanded outside the ramp (e.g. while libbarrett waits for
        // Shift-Activate): exit like an unhandled signal.  Ctrl-Z exits too, so a
        // stopped process cannot keep holding the CAN bus.
        const int sig=signal==SIGTSTP ? SIGTERM : signal;
        std::signal(sig,SIG_DFL);
        std::raise(sig);
        return;
    }
    interrupted.store(signal,std::memory_order_relaxed);
}
static bool number(const char* text,double& value) {
    char* end=NULL;value=std::strtod(text,&end);
    return end!=text && *end=='\0' && std::isfinite(value);
}
bool validate_args(int argc,char** argv) {
    std::setvbuf(stdout,NULL,_IONBF,0);  // prompts must show through a pipe (tee)
    if(argc==2 && !std::strcmp(argv[1],"--help")) {
        std::puts("Usage: lbc_torque_sign [joint=4] [maxNm=3] [rampSeconds=3] [stopRad=0.02] [--hold-j7]\n"
                  "--hold-j7 requires joint 8, |maxNm| <= 0.5 and stopRad <= 0.349066 (20 degrees).\n"
                  "With --hold-j7, all OTHER joints retain at most a 0.04 rad movement cutoff.\n"
                  "J7 holds its starting angle only during the J8 ramp; all other torques are zero.\n"
                  "stopRad is a relative torque cutoff, not a position target or brake; time limit is rampSeconds+0.5.\n"
                  "Both wrist torques turn off together when the test stops. Support the arm and wrist.");
        std::exit(0);
    }
    double parsedJoint=joint;
    if(argc<1 || argc>6 || (argc>1 && !number(argv[1],parsedJoint)) ||
            parsedJoint!=std::floor(parsedJoint) || parsedJoint<1 || parsedJoint>8 ||
            (argc>2 && !number(argv[2],maxNm)) || (argc>3 && !number(argv[3],rampS)) ||
            (argc>4 && !number(argv[4],stopRad)) ||
            (argc==6 && std::strcmp(argv[5],"--hold-j7")) ||
            std::fabs(maxNm)>40 || maxNm==0 || rampS<=0 || rampS>10 || stopRad<=0 || stopRad>.349066) {
        std::fprintf(stderr,"Invalid arguments; use --help.\n");return false;
    }
    joint=int(parsedJoint);holdJ7=argc==6;
    if((holdJ7 && (joint!=8 || std::fabs(maxNm)>.5)) || (!holdJ7 && stopRad>.2)) {
        std::fprintf(stderr,"--hold-j7 requires J8, |torque| <= 0.5 Nm and stopRad <= 0.349066; otherwise stopRad <= 0.2.\n");return false;
    }
    for(int sig:{SIGINT,SIGTERM,SIGHUP,SIGTSTP}) std::signal(sig,signalHandler);
    return true;
}

template<size_t DOF>
int wam_main(int, char**, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	if (joint > (int)DOF || (holdJ7 && DOF!=8)) {
		printf("Requested joint/hold option does not match this WAM.\n");
		return 1;
	}

	typedef TorqueSignRamp<DOF> Ramp;
	Ramp torque(joint - 1, maxNm, rampS, stopRad,&interrupted);
	const double otherStop=holdJ7 ? std::min(.04,stopRad):stopRad;
	torque.setOtherJointLimit(otherStop);
	wam.gravityCompensate(false);
	wam.idle();
	if(holdJ7) {
		const double kp=wam.jpController.getKp()[6],kd=wam.jpController.getKd()[6];
		const double configured=wam.jpController.getControlSignalLimit()[6];
		if(!std::isfinite(configured) || configured<=0) throw std::runtime_error("Missing J7 torque limit");
		const double limit=std::min(.5,configured);
		torque.setHoldJoint(6,kp,kd,limit);
		systems::connect(wam.jvOutput,torque.velocityInput);
		printf("J7 PD hold during the ramp: kp=%.2f kd=%.2f, cap=%.2f Nm. No move to zero.\n"
		       "Support the other joints. J7 hold releases when the test stops.\n",kp,kd,limit);
	}
	systems::connect(wam.jpOutput, torque.input);
	wam.trackReferenceSignal(torque.output);

	printf("Ramping J%d from 0 to %+.2f Nm over %.1f s; stops after %.6f rad (%.2f deg) on J%d, "
			"or %.3f rad on any OTHER joint.\n"
			"Torque turns off at the cutoff; this does not brake or hold the target angle. "
			"Time limit %.1f s. Starting in 3 s - hand on Shift-Idle.\n",
			joint,maxNm,rampS,stopRad,stopRad*180/M_PI,joint,otherStop,rampS+.5);
	sleep(3);
	try {
		if (pm.getSafetyModule()->pollMode() != SafetyModule::ACTIVE) {
			torque.stop(Ramp::SAFETY_IDLE);
		} else {
			rampActive.store(1,std::memory_order_relaxed);
			torque.start();
			while (torque.getStatus().reason == Ramp::RUNNING) {
				torque.refresh();
				if (pm.getSafetyModule()->getMode() != SafetyModule::ACTIVE) {
					torque.stop(Ramp::SAFETY_IDLE);
					break;
				}
				usleep(10000);
			}
		}
	} catch (const std::exception& e) {
		torque.stop(Ramp::ERROR);
		fprintf(stderr, "Torque test stopped: %s\n", e.what());
	}
	const typename Ramp::Status result = torque.getStatus();
	torque.stop(result.reason);
	rampActive.store(0,std::memory_order_relaxed);
	printf("Stopped (%s) at %+.2f Nm after %.2f s: J%d moved %+.4f rad.\n",
			Ramp::describe(result.reason), result.applied, result.elapsed, joint, result.moved);
	if(holdJ7) printf("Last J7 hold command: %+.3f Nm. J7 hold and J8 torque are now zero.\n",result.holdApplied);
	printf("Encoder-derived changes at stop:\n");
	for (size_t i=0; i<DOF; ++i) {
		printf("  J%zu: %+.5f rad (%+.2f deg)%s\n", i+1, result.delta[i],
				result.delta[i]*180.0/M_PI, int(i)==result.stopJoint ? " <-- motion limit" : "");
	}
	if (result.reason == Ramp::OTHER_MOTION) {
		printf("Requested J%d, but J%d reached the motion limit. Sign check is inconclusive.\n",
				joint, result.stopJoint+1);
	}
	if (result.reason == Ramp::MOTION) {
		printf("Encoder-derived direction: %s (torque was %s) -> %s command sign.\n",
				result.moved > 0 ? "positive" : "negative", maxNm > 0 ? "positive" : "negative",
				((result.moved > 0) == (maxNm > 0)) ? "agrees with" : "opposes");
		printf("Confirm the physical joint and direction by observation; encoders alone do not establish them.\n");
	} else if (result.reason == Ramp::TIME_LIMIT) {
		printf("Requested motion cutoff was not reached before the time limit.\n");
	}
	printf("Shift-Idle to finish.\n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return (result.reason == Ramp::ERROR || result.reason == Ramp::MAIN_TIMEOUT || result.reason == Ramp::OTHER_MOTION || result.reason==Ramp::INTERRUPTED) ? 1 : 0;
}

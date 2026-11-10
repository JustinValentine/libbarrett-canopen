// Bounded WAM8 gravity-fit diagnostic. Does not modify calibration or gains.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include "gravity_ramp_guard.h"

static_assert(ATOMIC_INT_LOCK_FREE==2,"Signal stop flag must be lock-free");
static std::atomic<int> interrupted{0};
static FILE* csv=NULL;
static std::string csvPath;
static bool extendedTrial=false;
static void interruptHandler(int signal) { interrupted.store(signal,std::memory_order_relaxed); }

// Parse arguments, open the recording and handle --help BEFORE any CAN access.
#define BARRETT_SMF_WAM8G
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

bool validate_args(int argc,char** argv) {
    if (argc==2 && !std::strcmp(argv[1],"--help")) {
        std::puts("Usage: ex04_gravity_ramp [--30-percent] [--log NEW_FILE.csv]\n"
                  "Default: 0 to 10% gravity over 3 s, hold 0.5 s.\n"
                  "--30-percent: hand-guided 30% trial; 10 s ramp, 60 s hold (70 s total).\n"
                  "The 30% trial has no added displacement, speed or diagnostic torque cutoffs.\n"
                  "Both trials then zero torque and request Idle.\n"
                  "Default 10% trial retains 0.020 rad, 0.15 rad/s and diagnostic torque cutoffs.\n"
                  "Both stop on bad feedback, timing or communication failure. No automatic restart.\n"
                  "Support the arm and wrist: partial gravity does not hold their weight.");
        std::exit(0);
    }
    bool haveLog=false;
    for(int i=1;i<argc;++i) {
        if(!std::strcmp(argv[i],"--30-percent") && !extendedTrial) extendedTrial=true;
        else if(!std::strcmp(argv[i],"--log") && !haveLog && i+1<argc && argv[i+1][0]!='-') {
            csvPath=argv[++i];haveLog=true;
        } else {
            std::fprintf(stderr,"Use --help; choose the default trial or --30-percent.\n");
            return false;
        }
    }
    if(!haveLog) csvPath="/tmp/wam-gravity-ramp-"+std::to_string(getpid())+".csv";
    const int fd=open(csvPath.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
    if (fd<0) { std::perror("Cannot create CSV"); return false; }
    csv=fdopen(fd,"w");
    if (!csv) { close(fd); std::perror("Cannot open CSV stream"); return false; }
    for(int sig : {SIGINT,SIGTERM,SIGHUP,SIGTSTP}) std::signal(sig,interruptHandler);
    return true;
}

static void writeSamples(const std::vector<GravityRampGuard::Sample>& samples) {
    std::fprintf(csv,"monotonic_s,elapsed_s,state,stop_joint,applied_fraction");
    for(const char* field : {"position_rad","velocity_rad_s","model_Nm","command_Nm","delta_rad"})
        for(int j=1;j<=8;++j) std::fprintf(csv,",J%d_%s",j,field);
    std::fprintf(csv,"\n");
    for(const auto& s:samples) {
        std::fprintf(csv,"%.9f,%.6f,%s,%d,%.6f",s.time,s.elapsed,GravityRampGuard::describe(s.reason),s.joint+1,s.fraction);
        for(const auto* values : {&s.position,&s.velocity,&s.model,&s.command,&s.delta})
            for(double x:*values) std::fprintf(csv,",%.9f",x);
        std::fprintf(csv,"\n");
    }
    if (std::fflush(csv) || std::ferror(csv)) throw std::runtime_error("CSV write failed");
}

int wam_main(int,char**,barrett::ProductManager& pm,barrett::systems::Wam8g& wam) {
    using namespace barrett;
    typedef GravityRampGuard Guard;
    SafetyModule* safety=pm.getSafetyModule();
    Guard guard(&interrupted,extendedTrial);
    // The unvalidated fit is observed only through the bounded diagnostic.
    // Never connect gravity.output directly to the WAM torque summer.
    wam.gravityCompensate(false);
    wam.idle();
    bool failed=false;
    try {
        if (safety->pollMode()!=SafetyModule::IDLE)
            throw std::runtime_error("Start this diagnostic in Shift-Idle.");
        systems::connect(wam.jpOutput,guard.positionInput);
        systems::connect(wam.gravity.output,guard.gravityInput);
        guard.connectOutputTo(wam.input);
        btsleep(0.1); // Obtain initialized feedback and a model sample, with zero output.
        if (interrupted) throw std::runtime_error("Interrupted before activation.");
        guard.arm();
        const auto initial=guard.snapshot();
        std::printf("Full model torque at start (Nm):");
        for(double t:initial.model) std::printf(" %+.3f",t);
        std::printf("\nSupport the arm AND wrist; keep Shift-Idle ready.\n"
                    "This trial provides only 0-%.0f%% gravity support.\n"
                    "After Shift-Activate: ramp for %.1f s, hold %.1f s, then stop.\n",
                    100*guard.maxFraction(),guard.rampSeconds(),guard.holdSeconds());
        if(guard.diagnosticLimitsEnabled()) {
            std::printf("Diagnostic torque caps (Nm):");
            for(size_t j=0;j<8;++j) std::printf(" %.2f",guard.torqueLimit(j));
            std::puts("\nAny joint: stop at 0.020 rad or 0.15 rad/s (20 ms estimate).");
        } else {
            std::puts("Hand-guided trial: no added displacement, speed or diagnostic torque cutoffs.\n"
                      "Compensation follows the measured pose at up to 30%. No position hold.");
        }
        std::puts(
                  "Ctrl-C or Ctrl-Z aborts; a stopped trial cannot restart.\n"
                  ">>> Please Shift-activate the WAM when supported.");
        std::fflush(stdout);
        while(guard.snapshot().reason==Guard::ARMED) {
            if (interrupted) { guard.stop(Guard::INTERRUPTED); break; }
            if (pm.getExecutionManager()->getError()) throw std::runtime_error("Control thread failed.");
            // Match normal startup retries only while commanded torque is zero.
            const auto mode=safety->pollMode();
            if (mode==SafetyModule::ACTIVE) { guard.start(); break; }
            if (mode!=SafetyModule::IDLE) { guard.stop(Guard::SAFETY_IDLE); break; }
            btsleep(0.05);
        }
        while(guard.snapshot().reason==Guard::RUNNING) {
            guard.refresh();
            if (interrupted) guard.stop(Guard::INTERRUPTED);
            if (pm.getExecutionManager()->getError()) throw std::runtime_error("Control thread failed.");
            if (safety->getMode()!=SafetyModule::ACTIVE) guard.stop(Guard::SAFETY_IDLE);
            btsleep(0.01);
        }
    } catch(const std::exception& e) {
        guard.stop(Guard::CONTROL_ERROR);
        std::fprintf(stderr,"Diagnostic stopped: %s\n",e.what());
        failed=true;
    }
    // Zero is latched in the control graph before any potentially blocking
    // safety-board access. Keep that graph connected while requesting Idle.
    const auto result=guard.snapshot();
    guard.stop(Guard::CONTROL_ERROR);
    bool idle=false;
    for(int attempt=0;attempt<3 && !idle;++attempt) {
        try {
            const auto mode=safety->getMode();
            if (mode==SafetyModule::ACTIVE) safety->setMode(SafetyModule::IDLE);
            const auto after=safety->getMode();
            idle=after==SafetyModule::IDLE || after==SafetyModule::ESTOP;
        } catch(const std::exception& e) { std::fprintf(stderr,"Idle check: %s\n",e.what()); }
        if (!idle) btsleep(0.05);
    }
    if (!idle) {
        std::fprintf(stderr,"Could not confirm Idle. Press Shift-Idle now.\n");
        failed=true;
    }
    std::printf("Stopped: %s",Guard::describe(result.reason));
    if (result.joint>=0) std::printf(" (J%d)",result.joint+1);
    std::printf("; %.3f s, peak %.2f%%. Torque gate is zero.\n",result.elapsed,100*result.peakFraction);
    std::puts("Keep the arm and wrist supported.");
    btsleep(0.01); // Include the zero-output stop sample if the control thread is running.
    try { writeSamples(guard.samples()); }
    catch(const std::exception& e) { std::fprintf(stderr,"%s\n",e.what()); failed=true; }
    std::fclose(csv); csv=NULL;
    std::printf("CSV: %s\n",csvPath.c_str());
    if (idle) std::puts("Drives are Idle/disabled. Trial finished; no automatic restart.");
    return failed || result.reason!=Guard::COMPLETE ? 1 : 0;
}

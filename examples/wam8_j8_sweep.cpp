// Position-controlled identity check: home -> DH zero -> J8 +10/-10/0 deg.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include "joint_sweep_guard.h"

static_assert(ATOMIC_INT_LOCK_FREE==2,"Signal flag must be lock-free");
static std::atomic<int> interrupted{0};
static FILE* csv=NULL;
static std::string csvPath;
static void signalHandler(int signal) { interrupted.store(signal,std::memory_order_relaxed); }
#define BARRETT_SMF_WAM8G
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

bool validate_args(int argc,char** argv) {
    if(argc==2 && !std::strcmp(argv[1],"--help")) {
        std::puts("Usage: wam8_j8_sweep [--log NEW_FILE.csv]\n"
                  "From the saved home: move all powered joints to DH zero, then J8 +10, -10, 0 degrees.\n"
                  "Passive J5 stays fixed; J7 stays commanded at zero during the sweep.\n"
                  "Gravity compensation OFF. Position PD with tracking/speed/torque cutoffs.\n"
                  "Holds zero at the end until Shift-Idle. Support the arm before any stop.");
        std::exit(0);
    }
    if(argc!=1 && !(argc==3 && !std::strcmp(argv[1],"--log"))) return false;
    csvPath=argc==3 ? argv[2] : "/tmp/wam8-j8-sweep-"+std::to_string(getpid())+".csv";
    int fd=open(csvPath.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0) { std::perror("CSV");return false; }
    csv=fdopen(fd,"w");if(!csv) { close(fd);return false; }
    for(int sig:{SIGINT,SIGTERM,SIGHUP,SIGTSTP}) std::signal(sig,signalHandler);
    return true;
}

using namespace barrett;
typedef JointSweepGuard Guard;
static bool monitor(ProductManager& pm,Guard& guard) {
    guard.refresh();
    if(interrupted) guard.stop(Guard::INTERRUPTED);
    if(pm.getExecutionManager()->getError()) throw std::runtime_error("Control thread failed");
    if(pm.getSafetyModule()->getMode()!=SafetyModule::ACTIVE) guard.stop(Guard::SAFETY_IDLE);
    return guard.snapshot().reason==Guard::RUNNING;
}
static bool moveAndSettle(ProductManager& pm,Guard& guard,JointSweepReference& reference,
        const Guard::jp_type& target,int phase,double speed,const char* label) {
    if(guard.snapshot().reason!=Guard::RUNNING) return false;
    std::printf("%s\n",label);std::fflush(stdout);
    // A blocked terminal must not permit another move after the gate trips.
    guard.refresh();if(guard.snapshot().reason!=Guard::RUNNING) return false;
    guard.setPhase(phase);
    const double duration=reference.move(target,speed);
    const double start=highResolutionSystemTime();double stableSince=0;
    while(monitor(pm,guard)) {
        const double now=highResolutionSystemTime();
        const auto sample=guard.snapshot();
        bool settled=now-start>=duration;
        for(int j=0;j<8;++j) if(j!=4)
            settled=settled && std::fabs(sample.error[j])<=std::acos(-1.0)/180 && std::fabs(sample.velocity[j])<=.03;
        if(settled) {
            if(stableSince==0) stableSince=now;
            if(now-stableSince>=1.5) return true;
        } else stableSince=0;
        if(now-start>duration+8) { guard.stop(Guard::NOT_SETTLED);return false; }
        btsleep(.01);
    }
    return false;
}

int wam_main(int,char**,ProductManager& pm,systems::Wam8g& wam) {
    wam.gravityCompensate(false);wam.idle();
    btsleep(.1);
    const Guard::jp_type initial=wam.getJointPositions();
    JointSweepReference reference(initial);
    Guard::jt_type caps=wam.jpController.getControlSignalLimit();
    const double maximum[8]={25,20,15,15,0,.75,.50,.50};
    for(int j=0;j<8;++j) {
        if(!std::isfinite(caps[j]) || (j!=4 && caps[j]<=0)) throw std::runtime_error("Missing position torque cap");
        caps[j]=std::min(caps[j],maximum[j]);
    }
    Guard guard(caps,&interrupted);
    bool completed=false,failed=false;
    std::string error;
    try {
        if(pm.getSafetyModule()->pollMode()!=SafetyModule::IDLE) throw std::runtime_error("Start in Shift-Idle");
        {
            BARRETT_SCOPED_LOCK(wam.getEmMutex());
            // Runtime-only changes: no integrator windup while waiting or capped.
            wam.jpController.setKi(Guard::jp_type(0.0));
            wam.jpController.resetIntegrator();
            wam.jpController.setControlSignalLimit(caps);
            systems::forceConnect(reference.output,wam.jpController.referenceInput);
            systems::connect(wam.jpOutput,guard.positionInput);
            systems::connect(reference.output,guard.referenceInput);
            systems::connect(wam.jpController.controlOutput,guard.torqueInput);
            guard.connectOutputTo(wam.input);
        }
        btsleep(.1);guard.arm();
        std::puts("Position test: all powered joints -> DH zero; J8 +10, -10, then 0 degrees.\n"
                  "Gravity compensation is OFF. J5 is passive.\n"
                  "Transit is slow (about one minute). Keep the full path clear and Shift-Idle ready.\n"
                  "An abort removes torque: be ready to support the arm and wrist.\n"
                  "At the end, position control holds zero until you Shift-Idle.\n"
                  ">>> Please Shift-activate the WAM.");std::fflush(stdout);
        while(guard.snapshot().reason==Guard::ARMED) {
            if(interrupted) { guard.stop(Guard::INTERRUPTED);break; }
            if(pm.getExecutionManager()->getError()) throw std::runtime_error("Control thread failed");
            // Use the normal libbarrett startup retry while the gate is zero.
            // Motion-time polling remains fail-stop in monitor().
            const auto mode=pm.getSafetyModule()->pollMode();
            if(mode==SafetyModule::ACTIVE) { guard.start();break; }
            if(mode!=SafetyModule::IDLE) { guard.stop(Guard::SAFETY_IDLE);break; }
            btsleep(.05);
        }
        Guard::jp_type target(0.0);target[4]=initial[4];
        if(moveAndSettle(pm,guard,reference,target,0,.10,"Moving all powered joints to DH zero...") ) {
            target[7]=10*std::acos(-1.0)/180;
            if(moveAndSettle(pm,guard,reference,target,1,.08,"J8 -> +10 degrees. Watch physical J8; J7 should remain still.")) {
                target[7]=-10*std::acos(-1.0)/180;
                if(moveAndSettle(pm,guard,reference,target,2,.08,"J8 -> -10 degrees.")) {
                    target[7]=0;
                    completed=moveAndSettle(pm,guard,reference,target,3,.08,"J8 -> 0 degrees.");
                }
            }
        }
        if(completed) {
            guard.setPhase(4);guard.finishRecording();
            std::puts("Sequence complete. Holding zero. Support the arm, then Shift-Idle to finish.\n"
                      "Encoder tracking alone does not confirm the physical joint identity.");std::fflush(stdout);
            while(monitor(pm,guard)) btsleep(.01);
        }
    } catch(const std::exception& e) { guard.stop(Guard::CONTROL_ERROR);error=e.what();failed=true; }
    const auto result=guard.snapshot();guard.stop(Guard::CONTROL_ERROR);
    bool idle=false;
    for(int attempt=0;attempt<3 && !idle;++attempt) {
        try {
            const auto mode=pm.getSafetyModule()->getMode();
            if(mode==SafetyModule::ACTIVE) pm.getSafetyModule()->setMode(SafetyModule::IDLE);
            const auto after=pm.getSafetyModule()->getMode();idle=after==SafetyModule::IDLE || after==SafetyModule::ESTOP;
        } catch(const std::exception& e) { error+=std::string("; Idle check: ")+e.what(); }
        if(!idle) btsleep(.05);
    }
    if(!idle) { std::fprintf(stderr,"Could not confirm Idle. Press Shift-Idle now.\n");failed=true; }
    if(!error.empty()) std::fprintf(stderr,"%s\n",error.c_str());
    std::printf("Stopped: %s; phase %d",Guard::describe(result.reason),result.phase);
    if(result.joint>=0) std::printf("; J%d",result.joint+1);
    std::printf(". Elapsed %.2f s.\n",result.elapsed);
    btsleep(.01);
    std::fprintf(csv,"monotonic_s,elapsed_s,state,phase,stop_joint");
    for(const char* name:{"position_rad","reference_rad","velocity_rad_s","command_Nm","error_rad"})
        for(int j=1;j<=8;++j) std::fprintf(csv,",J%d_%s",j,name);
    std::fprintf(csv,"\n");
    for(const auto& s:guard.samples()) {
        std::fprintf(csv,"%.9f,%.6f,%s,%d,%d",s.time,s.elapsed,Guard::describe(s.reason),s.phase,s.joint+1);
        for(const auto* values:{&s.position,&s.reference,&s.velocity,&s.command,&s.error})
            for(double x:*values) std::fprintf(csv,",%.9f",x);
        std::fprintf(csv,"\n");
    }
    if(std::fflush(csv) || std::ferror(csv)) { std::fprintf(stderr,"CSV write failed\n");failed=true; }
    std::fclose(csv);csv=NULL;std::printf("CSV: %s\n",csvPath.c_str());
    return failed || !completed || result.reason!=Guard::SAFETY_IDLE ? 1:0;
}

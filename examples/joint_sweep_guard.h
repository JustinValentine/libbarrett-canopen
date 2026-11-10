#ifndef BARRETT_EXAMPLES_JOINT_SWEEP_GUARD_H_
#define BARRETT_EXAMPLES_JOINT_SWEEP_GUARD_H_
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/helpers.h>

// Smooth joint-space references. Peak speed of the quintic blend is 1.875/T.
class JointSweepReference : public barrett::systems::System,
        public barrett::systems::SingleOutput<barrett::units::JointPositions<8>::type> {
public:
    typedef barrett::units::JointPositions<8>::type jp_type;
    explicit JointSweepReference(const jp_type& initial) :
        barrett::systems::System("JointSweepReference"),
        barrett::systems::SingleOutput<jp_type>(this),from(initial),target(initial),value(initial) {
        if (!initial.allFinite()) throw std::invalid_argument("Invalid initial position");
        this->outputValue->setData(&value);
    }
    ~JointSweepReference() { this->mandatoryCleanUp(); }
    double move(const jp_type& destination,double maxSpeed) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if (!destination.allFinite() || !std::isfinite(maxSpeed) || maxSpeed<=0 || maxSpeed>0.10)
            throw std::invalid_argument("Invalid diagnostic trajectory");
        from=value; target=destination;
        target[4]=from[4]; // J5 is mechanically fixed, not commanded to zero.
        duration=std::max(1.0,1.875*(target-from).cwiseAbs().maxCoeff()/maxSpeed);
        start=barrett::highResolutionSystemTime(); moving=true;
        return duration;
    }
protected:
    void operate() override {
        if (moving) {
            const double u=std::max(0.0,std::min(1.0,(barrett::highResolutionSystemTime()-start)/duration));
            const double blend=u*u*u*(10.0+u*(-15.0+6.0*u));
            value=from+(target-from)*blend;
            if(u>=1) { value=target; moving=false; }
        }
        this->outputValue->setData(&value);
    }
private:
    jp_type from,target,value;
    double start=0,duration=1;
    bool moving=false;
};

// Gate the existing position controller's output. No gravity input is used.
class JointSweepGuard : public barrett::systems::System,
        public barrett::systems::SingleOutput<barrett::units::JointTorques<8>::type> {
public:
    typedef barrett::units::JointPositions<8>::type jp_type;
    typedef barrett::units::JointTorques<8>::type jt_type;
    enum Reason { WAITING, ARMED, RUNNING, TRACKING_ERROR, SPEED, TORQUE_LIMIT,
                  INVALID_FEEDBACK, MAIN_TIMEOUT, LOOP_DELAY, SAFETY_IDLE,
                  INTERRUPTED, CONTROL_ERROR, NOT_SETTLED, LOG_FULL };
    struct Sample {
        double time=0,elapsed=0;
        Reason reason=WAITING;
        int joint=-1,phase=0;
        std::array<double,8> position{},reference{},velocity{},command{},error{};
    };
    barrett::systems::System::Input<jp_type> positionInput,referenceInput;
    barrett::systems::System::Input<jt_type> torqueInput;
    JointSweepGuard(const jt_type& caps,std::atomic<int>* interrupted) :
        barrett::systems::System("JointSweepGuard"),
        barrett::systems::SingleOutput<jt_type>(this),positionInput(this),referenceInput(this),
        torqueInput(this),caps(caps),outputTorque(0.0),anchor(0.0),interrupted(interrupted),records(10000) {
        if (!caps.allFinite() || caps[4]!=0) throw std::invalid_argument("Invalid torque caps");
        for(int j=0;j<8;++j) if(j!=4 && caps[j]<=0) throw std::invalid_argument("Missing torque cap");
        this->outputValue->setData(&outputTorque);
    }
    ~JointSweepGuard() { this->mandatoryCleanUp(); }
    // Call in Idle, with gravity and the supervisory controller disconnected.
    // Older Wam8g builds already occupy this input with externalTorques.output.
    // This exclusive diagnostic replaces that source with its zero-output gate.
    void connectOutputTo(barrett::systems::System::Input<jt_type>& input) {
        boost::lock_guard<barrett::thread::Mutex> inputLock(input.getEmMutex());
        boost::lock_guard<barrett::thread::Mutex> outputLock(this->getEmMutex());
        if(state.reason!=WAITING) throw std::logic_error("Connect the torque gate before arming");
        barrett::systems::forceConnect(this->output,input);
    }
    void arm() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        const double now=barrett::highResolutionSystemTime();
        if(state.reason!=WAITING || !valid || now-state.time>.01) throw std::runtime_error("No fresh feedback to arm");
        state.reason=ARMED;lastCycle=anchorTime=now;
        for(int j=0;j<8;++j) anchor[j]=state.position[j];
        records[count++]=state;
    }
    bool start() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if(state.reason!=ARMED) return false;
        const double now=barrett::highResolutionSystemTime();
        if(!valid || now-state.time>.01) { latch(INVALID_FEEDBACK);return false; }
        if(interrupted->load(std::memory_order_relaxed)) { latch(INTERRUPTED);return false; }
        state.reason=RUNNING;startTime=refreshTime=now;return true;
    }
    void refresh() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        const double now=barrett::highResolutionSystemTime();
        if(state.reason==RUNNING && now-refreshTime>=.05) latch(MAIN_TIMEOUT);
        refreshTime=now;
    }
    void stop(Reason reason) { BARRETT_SCOPED_LOCK(this->getEmMutex()); latch(reason); }
    void setPhase(int phase) { BARRETT_SCOPED_LOCK(this->getEmMutex());state.phase=phase; }
    void finishRecording() { BARRETT_SCOPED_LOCK(this->getEmMutex());recording=false; }
    Sample snapshot() const { BARRETT_SCOPED_LOCK(this->getEmMutex());return state; }
    std::vector<Sample> samples() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return std::vector<Sample>(records.begin(),records.begin()+count);
    }
    static bool stopped(Reason r) { return r!=WAITING && r!=ARMED && r!=RUNNING; }
    static const char* describe(Reason r) {
        switch(r) {
        case WAITING:return "waiting";case ARMED:return "armed";case RUNNING:return "running";
        case TRACKING_ERROR:return "tracking error";case SPEED:return "speed limit";
        case TORQUE_LIMIT:return "torque limit";case INVALID_FEEDBACK:return "invalid feedback or command";
        case MAIN_TIMEOUT:return "monitor stalled 50 ms";case LOOP_DELAY:return "control delay over 20 ms";
        case SAFETY_IDLE:return "safety board left ACTIVE";case INTERRUPTED:return "operator interrupt";
        case CONTROL_ERROR:return "control or communication error";case NOT_SETTLED:return "target did not settle";
        case LOG_FULL:return "sample buffer full";
        }
        return "unknown";
    }
protected:
    bool inputsValid() override { return true; }
    void operate() override {
        const double now=barrett::highResolutionSystemTime();state.time=now;
        outputTorque.setZero();state.command.fill(0);
        valid=positionInput.valueDefined() && referenceInput.valueDefined() && torqueInput.valueDefined();
        if(valid) valid=positionInput.getValue().allFinite() && referenceInput.getValue().allFinite() && torqueInput.getValue().allFinite();
        if(!valid) latch(INVALID_FEEDBACK);
        if(interrupted->load(std::memory_order_relaxed)) latch(INTERRUPTED);
        if(valid) {
            const auto& q=positionInput.getValue();const auto& ref=referenceInput.getValue();const auto& tau=torqueInput.getValue();
            for(int j=0;j<8;++j) {
                state.position[j]=q[j];state.reference[j]=ref[j];state.error[j]=ref[j]-q[j];
            }
            if(state.reason==ARMED || state.reason==RUNNING) {
                if(!std::isfinite(now) || now<lastCycle || now-lastCycle>.020) latch(LOOP_DELAY);
                for(int j=0;j<8;++j) {
                    // 3 degrees during the home-to-zero transit, 2 degrees for
                    // the sweep; passive J5 must stay within 1 degree throughout.
                    const double limit=(state.reason==ARMED ? .5 : j==4 ? 1.0 : state.phase==0 ? 3.0:2.0)*std::acos(-1.0)/180;
                    if(std::fabs(state.error[j])>=limit) latch(TRACKING_ERROR,j);
                }
                if(now-anchorTime>=.020) {
                    for(int j=0;j<8;++j) {
                        state.velocity[j]=(q[j]-anchor[j])/(now-anchorTime);
                        if(std::fabs(state.velocity[j])>=.25) latch(SPEED,j);
                    }
                    anchor=q;anchorTime=now;
                }
            }
            if(state.reason==RUNNING) {
                state.elapsed=now-startTime;
                if(now-refreshTime>=.05) latch(MAIN_TIMEOUT);
                for(int j=0;j<8;++j)
                    if(j!=4 && std::fabs(tau[j])>caps[j]+1e-9) latch(TORQUE_LIMIT,j);
                if(recording && count+1>=records.size()) latch(LOG_FULL);
                if(state.reason==RUNNING) for(int j=0;j<8;++j) {
                    outputTorque[j]=(j==4 ? 0:tau[j]);state.command[j]=outputTorque[j];
                }
            }
        }
        lastCycle=now;
        if(!terminalRecorded && count<records.size() &&
                (stopped(state.reason) || (recording && state.reason==RUNNING && now-lastRecord>=.020))) {
            records[count++]=state;lastRecord=now;terminalRecorded=stopped(state.reason);
        }
        this->outputValue->setData(&outputTorque);
    }
private:
    void latch(Reason r,int joint=-1) {
        if(!stopped(state.reason)) { state.reason=r;state.joint=joint; }
        outputTorque.setZero();state.command.fill(0);
    }
    jt_type caps,outputTorque;
    jp_type anchor;
    std::atomic<int>* interrupted;
    Sample state;
    bool valid=false,recording=true,terminalRecorded=false;
    double startTime=0,refreshTime=0,lastCycle=0,anchorTime=0,lastRecord=0;
    std::vector<Sample> records;
    size_t count=0;
};
#endif

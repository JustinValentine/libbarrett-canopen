#ifndef BARRETT_EXAMPLES_GRAVITY_RAMP_GUARD_H_
#define BARRETT_EXAMPLES_GRAVITY_RAMP_GUARD_H_

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <vector>
#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/helpers.h>

// Default: stationary 10% diagnostic. Extended: operator-guided 30% trial.
// All torque gating and stop latches execute in the control graph. No SDO,
// allocation, file I/O or terminal I/O occurs in operate().
class GravityRampGuard : public barrett::systems::System,
        public barrett::systems::SingleOutput<barrett::units::JointTorques<8>::type> {
public:
    typedef barrett::units::JointPositions<8>::type jp_type;
    typedef barrett::units::JointTorques<8>::type jt_type;
    enum Reason { WAITING, ARMED, RUNNING, COMPLETE, MOTION, SPEED, TORQUE_LIMIT,
                  INVALID_FEEDBACK, MAIN_TIMEOUT, LOOP_DELAY, SAFETY_IDLE,
                  INTERRUPTED, CONTROL_ERROR, LOG_FULL };
    struct Sample {
        double time = 0, elapsed = 0, fraction = 0, peakFraction = 0;
        Reason reason = WAITING;
        int joint = -1;
        std::array<double,8> position{}, velocity{}, model{}, command{}, delta{};
    };
    barrett::systems::System::Input<jp_type> positionInput;
    barrett::systems::System::Input<jt_type> gravityInput;

    double maxFraction() const { return extendedTrial ? 0.30 : 0.10; }
    double rampSeconds() const { return extendedTrial ? 10.0 : 3.0; }
    double holdSeconds() const { return extendedTrial ? 60.0 : 0.5; }
    bool diagnosticLimitsEnabled() const { return !extendedTrial; }
    static double motionLimit() { return 0.020; }
    static double speedLimit() { return 0.15; }
    double torqueLimit(size_t j) const {
        const double limits[8] = {2.5, 2.0, 1.5, 1.5, 0.0, 0.20, 0.10, 0.10};
        // These caps apply only to the default stationary diagnostic.
        return limits[j];
    }
    explicit GravityRampGuard(std::atomic<int>* interrupted = NULL, bool extendedTrial = false) :
        barrett::systems::System("GravityRampGuard"),
        barrett::systems::SingleOutput<jt_type>(this),
        positionInput(this), gravityInput(this), interrupted(interrupted), extendedTrial(extendedTrial),
        outputTorque(0.0), initial(0.0), speedAnchor(0.0),
        // Preallocate for the full 70-second trial, with margin at up to 1 kHz.
        records(extendedTrial ? 80000 : 4096) {
        this->outputValue->setData(&outputTorque);
    }
    ~GravityRampGuard() { this->mandatoryCleanUp(); }
    // In Idle, replace the source already connected by older Wam8g builds.
    // Only this diagnostic's initially-zero output should feed the torque input.
    void connectOutputTo(barrett::systems::System::Input<jt_type>& input) {
        boost::lock_guard<barrett::thread::Mutex> inputLock(input.getEmMutex());
        boost::lock_guard<barrett::thread::Mutex> outputLock(this->getEmMutex());
        if(state.reason!=WAITING) throw std::logic_error("Connect the torque gate before arming");
        barrett::systems::forceConnect(this->output,input);
    }
    void arm() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        const double now = barrett::highResolutionSystemTime();
        if (state.reason != WAITING || !valid || now-state.time > 0.01)
            throw std::runtime_error("Cannot arm without fresh, finite feedback.");
        for (size_t j=0;j<8;++j) initial[j]=speedAnchor[j]=state.position[j];
        state.reason=ARMED;
        anchorTime=lastCycle=now;
        state.delta.fill(0.0);
        recording=true;
        records[recordCount++]=state;
    }
    bool start() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        const double now=barrett::highResolutionSystemTime();
        if (state.reason != ARMED) return false;
        if (!valid || now-state.time > 0.01) { latch(INVALID_FEEDBACK); return false; }
        if (interrupted && interrupted->load(std::memory_order_relaxed)) { latch(INTERRUPTED); return false; }
        state.reason=RUNNING;
        startTime=refreshTime=now;
        recording=true;
        return true;
    }
    void refresh() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        const double now=barrett::highResolutionSystemTime();
        if (state.reason == RUNNING && now-refreshTime >= 0.05) latch(MAIN_TIMEOUT);
        refreshTime=now; // A late refresh never clears a stop.
    }
    void stop(Reason reason) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        latch(reason);
    }
    Sample snapshot() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return state;
    }
    std::vector<Sample> samples() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return std::vector<Sample>(records.begin(),records.begin()+recordCount);
    }
    static bool stopped(Reason r) { return r!=WAITING && r!=ARMED && r!=RUNNING; }
    static const char* describe(Reason r) {
        switch(r) {
        case WAITING: return "waiting for feedback";
        case ARMED: return "waiting for Shift-Activate";
        case RUNNING: return "ramping or holding";
        case COMPLETE: return "gravity trial complete";
        case MOTION: return "joint displacement limit";
        case SPEED: return "joint speed limit (20 ms estimate)";
        case TORQUE_LIMIT: return "diagnostic torque limit";
        case INVALID_FEEDBACK: return "missing or nonfinite feedback/model";
        case MAIN_TIMEOUT: return "monitor thread stalled for 50 ms";
        case LOOP_DELAY: return "control update delayed over 20 ms";
        case SAFETY_IDLE: return "safety board left ACTIVE";
        case INTERRUPTED: return "operator interrupt";
        case CONTROL_ERROR: return "control or communication error";
        case LOG_FULL: return "sample buffer full";
        }
        return "unknown";
    }
protected:
    // Undefined upstream data must produce a latched zero, not silently skip
    // operate() and resume torque when that input reappears.
    bool inputsValid() override { return true; }
    void operate() override {
        const double now=barrett::highResolutionSystemTime();
        state.time=now;
        outputTorque.setZero();
        state.command.fill(0.0);
        state.fraction=0;
        valid=positionInput.valueDefined() && gravityInput.valueDefined();
        if (valid) valid=positionInput.getValue().allFinite() && gravityInput.getValue().allFinite();
        if (!valid) latch(INVALID_FEEDBACK);
        if (interrupted && interrupted->load(std::memory_order_relaxed)) latch(INTERRUPTED);
        if (valid) {
            const jp_type& q=positionInput.getValue();
            const jt_type& g=gravityInput.getValue();
            for(size_t j=0;j<8;++j) {
                state.position[j]=q[j]; state.model[j]=g[j];
                state.delta[j]=q[j]-initial[j];
            }
            if (state.reason==ARMED || state.reason==RUNNING) {
                if (!std::isfinite(now) || now<lastCycle || now-lastCycle>0.020) latch(LOOP_DELAY);
                if(diagnosticLimitsEnabled())
                    for(size_t j=0;j<8;++j)
                        if (std::fabs(state.delta[j])>=motionLimit()) latch(MOTION,int(j));
                // Use a short position window to avoid false speed trips from
                // single encoder-count jitter, including passive J5.
                if (now-anchorTime>=0.020) {
                    for(size_t j=0;j<8;++j) {
                        state.velocity[j]=(q[j]-speedAnchor[j])/(now-anchorTime);
                        if (diagnosticLimitsEnabled() && std::fabs(state.velocity[j])>=speedLimit()) latch(SPEED,int(j));
                    }
                    speedAnchor=q; anchorTime=now;
                }
            }
            if (state.reason==RUNNING) {
                state.elapsed=now-startTime;
                if (now-refreshTime>=0.05) latch(MAIN_TIMEOUT);
                if (state.elapsed>=rampSeconds()+holdSeconds()) latch(COMPLETE);
                const double fraction=maxFraction()*std::min(1.0,state.elapsed/rampSeconds());
                if(diagnosticLimitsEnabled())
                    for(size_t j=0;j<8;++j)
                        if (j!=4 && std::fabs(fraction*g[j])>torqueLimit(j)) latch(TORQUE_LIMIT,int(j));
                if (recordCount+1>=records.size()) latch(LOG_FULL);
                if (state.reason==RUNNING) {
                    state.fraction=fraction;
                    state.peakFraction=std::max(state.peakFraction,fraction);
                    for(size_t j=0;j<8;++j) {
                        outputTorque[j]=(j==4 ? 0.0 : fraction*g[j]);
                        state.command[j]=outputTorque[j];
                    }
                }
            }
        }
        lastCycle=now;
        if (recording && state.reason!=ARMED && !terminalRecorded && recordCount<records.size()) {
            records[recordCount++]=state;
            terminalRecorded=stopped(state.reason);
        }
        this->outputValue->setData(&outputTorque);
    }
private:
    void latch(Reason reason,int joint=-1) {
        if (!stopped(state.reason)) { state.reason=reason; state.joint=joint; }
        outputTorque.setZero(); state.command.fill(0.0); state.fraction=0;
    }
    std::atomic<int>* interrupted;
    const bool extendedTrial;
    jt_type outputTorque;
    jp_type initial,speedAnchor;
    Sample state;
    bool valid=false,recording=false,terminalRecorded=false;
    double startTime=0,refreshTime=0,lastCycle=0,anchorTime=0;
    std::vector<Sample> records;
    size_t recordCount=0;
};
#endif

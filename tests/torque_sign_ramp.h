#ifndef BARRETT_TESTS_TORQUE_SIGN_RAMP_H_
#define BARRETT_TESTS_TORQUE_SIGN_RAMP_H_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>

// Motion/time limits run in the control graph, independently of the main
// thread's safety-board SDO polls. A missed main-thread refresh latches zero;
// a late SDO reply cannot restart the ramp.
template<size_t DOF>
class TorqueSignRamp : public barrett::systems::System,
        public barrett::systems::SingleInput<typename barrett::units::JointPositions<DOF>::type>,
        public barrett::systems::SingleOutput<typename barrett::units::JointTorques<DOF>::type> {
public:
    typedef typename barrett::units::JointPositions<DOF>::type jp_type;
    typedef typename barrett::units::JointTorques<DOF>::type jt_type;
    typedef typename barrett::units::JointVelocities<DOF>::type jv_type;
    enum Reason { WAITING, RUNNING, MOTION, OTHER_MOTION, TIME_LIMIT, MAIN_TIMEOUT, SAFETY_IDLE, ERROR, INTERRUPTED };
    struct Status {
        Reason reason;
        double elapsed, applied, moved;
        jp_type delta;
        int stopJoint;
        double holdApplied;
    };
    barrett::systems::System::Input<jv_type> velocityInput;

    TorqueSignRamp(size_t joint, double cap, double seconds, double distance,
            std::atomic<int>* interrupted=NULL) :
        barrett::systems::System("TorqueSignRamp"),
        barrett::systems::SingleInput<jp_type>(this),
        barrett::systems::SingleOutput<jt_type>(this),
        velocityInput(this),joint(joint), cap(cap), seconds(seconds), distance(distance),otherDistance(distance),
        startTime(0), refreshTime(0), initial(0.0), torque(0.0),
        status{WAITING, 0, 0, 0, jp_type(0.0), -1, 0},interrupted(interrupted) { this->outputValue->setData(&torque); }
    ~TorqueSignRamp() { this->mandatoryCleanUp(); }

    // Increasing the test-axis travel must not widen held/other-axis stops.
    void setOtherJointLimit(double limit) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if(status.reason!=WAITING || !std::isfinite(limit) || limit<=0 || limit>distance)
            throw std::invalid_argument("Invalid other-joint motion limit");
        otherDistance=limit;
    }
    void setHoldJoint(size_t index,double kp,double kd,double limit) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if(status.reason!=WAITING || index>=DOF || index==joint ||
                !std::isfinite(kp) || kp<=0 || !std::isfinite(kd) || kd<0 ||
                !std::isfinite(limit) || limit<=0)
            throw std::invalid_argument("Invalid hold-joint settings");
        holdJoint=int(index);holdKp=kp;holdKd=kd;holdLimit=limit;
    }
    void start() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if(interrupted && interrupted->load(std::memory_order_relaxed)) { stop(INTERRUPTED);return; }
        if (status.reason != WAITING || !this->input.valueDefined())
            throw std::logic_error("Torque ramp cannot start without position feedback.");
        initial = this->input.getValue();
        if (!initial.allFinite()) throw std::runtime_error("Invalid joint position.");
        if(holdJoint>=0 && (!velocityInput.valueDefined() || !velocityInput.getValue().allFinite()))
            throw std::runtime_error("Joint hold needs valid velocity feedback.");
        startTime = refreshTime = barrett::highResolutionSystemTime();
        status = Status{RUNNING, 0, 0, 0, jp_type(0.0), -1, 0};
    }
    void refresh() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        const double now = barrett::highResolutionSystemTime();
        if (status.reason == RUNNING && now - refreshTime >= 0.05) {
            status.reason = MAIN_TIMEOUT;
            torque.setZero();
        }
        refreshTime = now;
    }
    void stop(Reason reason) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if (status.reason == RUNNING || status.reason == WAITING) status.reason = reason;
        torque.setZero();
    }
    Status getStatus() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return status;
    }
    static const char* describe(Reason reason) {
        switch (reason) {
        case MOTION: return "requested joint reached motion limit";
        case OTHER_MOTION: return "another joint reached motion limit";
        case TIME_LIMIT: return "torque cap and time limit reached";
        case MAIN_TIMEOUT: return "main/status thread stalled for 50 ms";
        case SAFETY_IDLE: return "safety board left ACTIVE";
        case ERROR: return "communication/feedback error";
        case INTERRUPTED: return "operator interrupt";
        default: return "not running";
        }
    }
protected:
    // Missing input must run the zero-output latch, not skip an update and
    // resume the previous command if the connection later recovers.
    bool inputsValid() override { return true; }
    void operate() override {
        torque.setZero();
        if(interrupted && interrupted->load(std::memory_order_relaxed)) stop(INTERRUPTED);
        if(status.reason==RUNNING && (!this->input.valueDefined() ||
                (holdJoint>=0 && (!velocityInput.valueDefined() || !velocityInput.getValue().allFinite()))))
            status.reason=ERROR;
        if (status.reason == RUNNING) {
            const double now = barrett::highResolutionSystemTime();
            status.elapsed = now - startTime;
            status.delta = this->input.getValue() - initial;
            status.moved = status.delta[joint];
            if (!status.delta.allFinite()) status.reason = ERROR;
            else {
                // Check other axes first: simultaneous unintended motion
                // must not be reported as a successful sign check.
                double largest = otherDistance;
                for (size_t i=0; i<DOF; ++i) {
                    if (i != joint && std::fabs(status.delta[i]) >= largest) {
                        largest = std::fabs(status.delta[i]);
                        status.stopJoint = int(i);
                        status.reason = OTHER_MOTION;
                    }
                }
                if (status.reason == RUNNING && std::fabs(status.moved) >= distance) {
                    status.stopJoint = int(joint);
                    status.reason = MOTION;
                }
                if (status.reason == RUNNING && now - refreshTime >= 0.05) status.reason = MAIN_TIMEOUT;
                if (status.reason == RUNNING && status.elapsed >= seconds + 0.5) status.reason = TIME_LIMIT;
            }
            if (status.reason == RUNNING) {
                status.applied = cap * std::min(1.0, status.elapsed / seconds);
                torque[joint] = status.applied;
                if(holdJoint>=0) {
                    const double hold=-holdKp*status.delta[holdJoint]-holdKd*velocityInput.getValue()[holdJoint];
                    if(!std::isfinite(hold)) { status.reason=ERROR;torque.setZero(); }
                    else {
                        status.holdApplied=std::max(-holdLimit,std::min(holdLimit,hold));
                        torque[holdJoint]=status.holdApplied;
                    }
                }
            }
        }
        this->outputValue->setData(&torque);
    }
private:
    size_t joint;
    double cap, seconds, distance, otherDistance, startTime, refreshTime;
    jp_type initial;
    jt_type torque;
    Status status;
    std::atomic<int>* interrupted;
    int holdJoint=-1;
    double holdKp=0,holdKd=0,holdLimit=0;
};

#endif

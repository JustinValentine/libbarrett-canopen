// Exercise the actual reference -> PD -> torque gate graph without hardware.
#include <cassert>
#include <csignal>
#include <iostream>
#include <limits>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/exposed_output.h>
#include <barrett/systems/pid_controller.h>
#include "../examples/joint_sweep_guard.h"

namespace barrett {
double testTime=1;
double highResolutionSystemTime() { return testTime; }
detail::LogFormatter logMessage(const std::string& s,bool output) { return detail::LogFormatter(s,output); }
void detail::LogFormatter::print() { printed=true; }
}
using namespace barrett;
typedef JointSweepGuard Guard;
static Guard::jt_type caps() {
    Guard::jt_type result;
    result<<25,20,15,15,0,.75,.5,.5;
    return result;
}
class Sink : public systems::System, public systems::SingleInput<Guard::jt_type> {
public:
    Guard::jt_type value;
    Sink(systems::ExecutionManager& em) : systems::System("Sink"),systems::SingleInput<Guard::jt_type>(this),value(0.0) { em.startManaging(*this); }
    ~Sink() { mandatoryCleanUp(); }
    void operate() override { value=input.getValue(); }
};
struct Trial {
    systems::ManualExecutionManager em{.002};
    systems::ExposedOutput<Guard::jp_type> position{Guard::jp_type(0.0)},reference{Guard::jp_type(0.0)};
    systems::ExposedOutput<Guard::jt_type> torque{Guard::jt_type(.1)};
    systems::ExposedOutput<Guard::jt_type> previousSource{Guard::jt_type(123.0)};
    std::atomic<int> signal{0};
    Guard guard{caps(),&signal};
    Sink sink{em};
    Trial() {
        systems::connect(position.output,guard.positionInput);
        systems::connect(reference.output,guard.referenceInput);
        systems::connect(torque.output,guard.torqueInput);
        // Reproduce opus: the destination is already occupied by another source.
        systems::connect(previousSource.output,sink.input);
        guard.connectOutputTo(sink.input);
        tick();guard.arm();
    }
    void tick(double dt=.002,bool refresh=true) {
        testTime+=dt;if(refresh)guard.refresh();em.runExecutionCycle();
    }
    void begin() { assert(guard.start());tick(); }
    void latched(Guard::Reason reason) {
        assert(guard.snapshot().reason==reason && sink.value.norm()==0);
        assert(!guard.start());
        for(int i=0;i<20;++i) tick();
        assert(guard.snapshot().reason==reason && sink.value.norm()==0);
        const auto log=guard.samples();
        assert(log.back().reason==reason);
        for(double x:log.back().command)assert(x==0);
    }
};
static void testSequence(bool occupied) {
    systems::ManualExecutionManager em(.002);
    Guard::jp_type home;
    home<<.011395,-1.990117,.003375,3.040944,.009204,0,-1.628581,-.025796;
    systems::ExposedOutput<Guard::jp_type> position(home);
    systems::ExposedOutput<Guard::jt_type> previousSource{Guard::jt_type(123.0)};
    JointSweepReference reference(home);
    systems::PIDController<Guard::jp_type,Guard::jt_type> pd;
    Guard::jp_type kp,kd;
    kp<<900,2500,600,300,0,20,10,10;
    kd<<10,20,10,2.5,0,.1,.1,.1;
    pd.setKp(kp);pd.setKd(kd);pd.setKi(Guard::jp_type(0.0));pd.setControlSignalLimit(caps());
    std::atomic<int> signal{0};Guard guard(caps(),&signal);Sink sink(em);
    systems::connect(position.output,pd.feedbackInput);
    systems::connect(reference.output,pd.referenceInput);
    systems::connect(position.output,guard.positionInput);
    systems::connect(reference.output,guard.referenceInput);
    systems::connect(pd.controlOutput,guard.torqueInput);
    if(occupied)systems::connect(previousSource.output,sink.input);
    guard.connectOutputTo(sink.input);
    testTime+=.002;em.runExecutionCycle();guard.arm();
    for(int i=0;i<100;++i){testTime+=.002;em.runExecutionCycle();assert(sink.value.norm()==0);}
    // Once armed, connecting another torque input must not be allowed.
    bool rejected=false;
    try{guard.connectOutputTo(sink.input);}catch(const std::logic_error&){rejected=true;}
    assert(rejected);
    assert(guard.start());
    Guard::jp_type previous=home;
    for(int phase=0;phase<4;++phase) {
        Guard::jp_type target(0.0);
        // Even a different passive-joint target must be ignored by the reference.
        target[4]=99;
        target[7]=(phase==1 ? 10:phase==2 ? -10:0)*std::acos(-1.0)/180;
        guard.setPhase(phase);
        const double speed=phase==0 ? .10:.08;
        const double duration=reference.move(target,speed);
        const int ticks=static_cast<int>(std::ceil((duration+1.6)/.002));
        for(int i=0;i<ticks;++i) {
            position.setValue(previous); // Ideal following plant, one cycle behind.
            testTime+=.002;guard.refresh();em.runExecutionCycle();
            const auto s=guard.snapshot();assert(s.reason==Guard::RUNNING);
            for(int j=0;j<8;++j) {
                assert(std::fabs(s.reference[j]-previous[j])<=speed*.002+1e-10);
                assert(std::fabs(sink.value[j])<=caps()[j]);
                if(phase>0 && j!=7)assert(s.reference[j]==(j==4 ? home[4]:0));
                previous[j]=s.reference[j];
            }
            assert(previous[4]==home[4] && sink.value[4]==0);
        }
        target[4]=home[4];assert((previous-target).norm()==0);
    }
    // Final zero holding can continue indefinitely without filling the log.
    const size_t recorded=guard.samples().size();guard.finishRecording();
    for(int i=0;i<110000;++i){position.setValue(previous);testTime+=.002;guard.refresh();em.runExecutionCycle();}
    assert(guard.snapshot().reason==Guard::RUNNING && guard.samples().size()==recorded);
    guard.stop(Guard::SAFETY_IDLE);testTime+=.002;em.runExecutionCycle();
    assert(sink.value.norm()==0 && guard.samples().back().reason==Guard::SAFETY_IDLE);
}
int main() {
    testSequence(false);testSequence(true);
    // A startup MODE retry can block the main thread for over a second.
    // Fresh control cycles must keep torque zero throughout, then permit a
    // confirmed activation. Motion-time stalls are still checked below.
    { Trial t;for(int i=0;i<650;++i){t.tick(.002,false);assert(t.sink.value.norm()==0);}
      assert(t.guard.snapshot().reason==Guard::ARMED);t.begin(); }
    { Trial t;for(int i=0;i<100;++i)t.tick();assert(t.sink.value.norm()==0);t.begin();assert(t.sink.value[4]==0); }
    // Every joint, including held J7 and passive J5, is monitored for error.
    for(int phase=0;phase<2;++phase)for(int j=0;j<8;++j) {
        Trial t;t.begin();t.guard.setPhase(phase);
        Guard::jp_type q(0.0);q[j]=(j==4 ? 1.01:phase ? 2.01:3.01)*std::acos(-1.0)/180;
        t.position.setValue(q);t.tick();assert(t.guard.snapshot().joint==j);t.latched(Guard::TRACKING_ERROR);
    }
    { Trial t;Guard::jp_type q(0.0);q[7]=.01;t.position.setValue(q);t.tick();t.latched(Guard::TRACKING_ERROR); }
    for(int j=0;j<8;++j) {
        Trial t;t.begin();
        for(int i=1;i<=15 && t.guard.snapshot().reason==Guard::RUNNING;++i) {
            Guard::jp_type q(0.0);q[j]=(j%2 ? -1:1)*i*.0006;t.position.setValue(q);t.tick();
        }
        t.latched(Guard::SPEED);
    }
    // All three upstream inputs must be defined and finite; stops never resume.
    for(int source=0;source<3;++source) {
        Trial t;t.begin();
        if(source==0)systems::disconnect(t.guard.positionInput);
        if(source==1)systems::disconnect(t.guard.referenceInput);
        if(source==2)systems::disconnect(t.guard.torqueInput);
        t.tick();t.latched(Guard::INVALID_FEEDBACK);
    }
    for(int source=0;source<3;++source) {
        Trial t;t.begin();Guard::jp_type q(0.0);q[7]=std::numeric_limits<double>::quiet_NaN();
        if(source==0)t.position.setValue(q);
        if(source==1)t.reference.setValue(q);
        if(source==2){Guard::jt_type tau(0.0);tau[6]=std::numeric_limits<double>::infinity();t.torque.setValue(tau);}
        t.tick();t.latched(Guard::INVALID_FEEDBACK);
    }
    for(int j=0;j<8;++j)if(j!=4) {
        Trial t;t.begin();auto tau=caps();tau[j]*=1.01;t.torque.setValue(tau);t.tick();t.latched(Guard::TORQUE_LIMIT);
    }
    {Trial t;t.begin();for(int i=0;i<30;++i)t.tick(.002,false);t.latched(Guard::MAIN_TIMEOUT);}
    {Trial t;t.begin();testTime+=.06;t.guard.refresh();t.tick();t.latched(Guard::MAIN_TIMEOUT);}
    {Trial t;t.begin();t.tick(.021);t.latched(Guard::LOOP_DELAY);}
    {Trial t;t.begin();t.signal=SIGTSTP;t.tick();t.latched(Guard::INTERRUPTED);}
    {Trial t;t.signal=SIGINT;assert(!t.guard.start());t.tick();t.latched(Guard::INTERRUPTED);}
    for(auto reason:{Guard::NOT_SETTLED,Guard::CONTROL_ERROR,Guard::SAFETY_IDLE}) {
        Trial t;t.begin();t.guard.stop(reason);t.tick();t.latched(reason);
    }
    std::cout<<"PASS: home/zero/+10/-10/zero reference and PD graph, speed bounds, passive masking, held-J7 errors, all-axis stops, invalid data, torque caps, watchdogs, signals and final holding.\n";
}

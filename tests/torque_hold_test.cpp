// Existing torque ramp plus optional single-joint PD hold, without CAN.
#include <cassert>
#include <csignal>
#include <iostream>
#include <limits>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/exposed_output.h>
#include "torque_sign_ramp.h"
namespace barrett {
double testTime=1;
double highResolutionSystemTime(){return testTime;}
detail::LogFormatter logMessage(const std::string& s,bool output){return detail::LogFormatter(s,output);}
void detail::LogFormatter::print(){printed=true;}
}
using namespace barrett;
typedef TorqueSignRamp<8> Ramp;
class Sink:public systems::System,public systems::SingleInput<Ramp::jt_type> {
public:
    Ramp::jt_type value{0.0};
    Sink(systems::ExecutionManager& em):systems::System("Sink"),systems::SingleInput<Ramp::jt_type>(this){em.startManaging(*this);}
    ~Sink(){mandatoryCleanUp();}
    void operate() override{value=input.getValue();}
};
struct Trial {
    systems::ManualExecutionManager em{.002};
    Ramp::jp_type initial{.1};
    systems::ExposedOutput<Ramp::jp_type> position{initial};
    systems::ExposedOutput<Ramp::jv_type> velocity{Ramp::jv_type(0.0)};
    std::atomic<int> signal{0};
    Ramp ramp;
    Sink sink{em};
    Trial(bool hold=true,double sign=1,double travel=.04):ramp(7,sign*.3,3,travel,&signal){
        systems::connect(position.output,ramp.input);
        if(hold){ramp.setHoldJoint(6,10,.1,.5);ramp.setOtherJointLimit(std::min(.04,travel));systems::connect(velocity.output,ramp.velocityInput);}
        systems::connect(ramp.output,sink.input);tick();assert(sink.value.norm()==0);
    }
    void tick(bool refresh=true){testTime+=.002;if(refresh)ramp.refresh();em.runExecutionCycle();}
    void begin(){ramp.start();tick();}
    void stopped(Ramp::Reason reason){
        assert(ramp.getStatus().reason==reason && sink.value.norm()==0);
        position.setValue(initial);velocity.setValue(Ramp::jv_type(0.0));
        for(int i=0;i<30;++i)tick();
        assert(ramp.getStatus().reason==reason && sink.value.norm()==0);
    }
};
int main(){
    // Default mode still emits only the requested torque, without velocity input.
    {Trial t(false);t.begin();for(int i=0;i<50;++i)t.tick();
     assert(t.sink.value[7]>0);for(int j=0;j<7;++j)assert(t.sink.value[j]==0);}
    // Holding captures the current angle, never DH zero; both J8 torque signs.
    for(double sign:{-1.0,1.0}){
        Trial t(true,sign);for(int i=0;i<100;++i)t.tick();assert(t.sink.value.norm()==0);t.begin();
        assert(t.sink.value[6]==0);
        auto q=t.initial;q[6]+=.01;t.position.setValue(q);
        Ramp::jv_type v(0.0);v[6]=.02;t.velocity.setValue(v);t.tick();
        assert(std::fabs(t.sink.value[6]-(-.102))<1e-10);
        assert(sign*t.sink.value[7]>0 && std::fabs(t.sink.value[7])<=.3);
        for(int j=0;j<6;++j)assert(t.sink.value[j]==0); // includes passive J5
        q[6]=t.initial[6]-.01;v[6]=-.02;t.position.setValue(q);t.velocity.setValue(v);t.tick();
        assert(std::fabs(t.sink.value[6]-.102)<1e-10);
        v[6]=-10;t.velocity.setValue(v);t.tick();assert(t.sink.value[6]==.5);
        q[7]+=.041;t.position.setValue(q);t.tick();t.stopped(Ramp::MOTION);
    }
    // Motion of the held joint or any other axis stops BOTH wrist commands.
    for(int j=0;j<7;++j){Trial t;t.begin();auto q=t.initial;q[j]+=.041;t.position.setValue(q);t.tick();t.stopped(Ramp::OTHER_MOTION);}
    // 20-degree J8 travel keeps the other seven axes at their original cutoff.
    for(double sign:{-1.0,1.0}) {
        Trial t(true,sign,.349066);t.begin();
        auto q=t.initial;q[7]+=sign*.34;t.position.setValue(q);t.tick();
        assert(t.ramp.getStatus().reason==Ramp::RUNNING);
        q[7]=t.initial[7]+sign*.350;t.position.setValue(q);t.tick();t.stopped(Ramp::MOTION);
    }
    for(int j=0;j<7;++j){Trial t(true,1,.349066);t.begin();auto q=t.initial;q[j]+=.041;t.position.setValue(q);t.tick();t.stopped(Ramp::OTHER_MOTION);}
    // If the target and another axis reach their limits together, abort for
    // the other axis; do not report a successful single-axis motion test.
    {Trial t(true,1,.349066);t.begin();auto q=t.initial;q[7]+=.35;q[6]+=.041;t.position.setValue(q);t.tick();t.stopped(Ramp::OTHER_MOTION);}
    {Trial t(true,1,.349066);t.begin();for(int i=0;i<1800;++i)t.tick();t.stopped(Ramp::TIME_LIMIT);}
    {Trial t;t.begin();for(int i=0;i<1800;++i)t.tick();t.stopped(Ramp::TIME_LIMIT);}
    {Trial t;t.begin();for(int i=0;i<30;++i)t.tick(false);t.stopped(Ramp::MAIN_TIMEOUT);}
    {Trial t;t.begin();testTime+=.06;t.ramp.refresh();t.tick();t.stopped(Ramp::MAIN_TIMEOUT);}
    {Trial t;t.begin();t.signal=SIGTSTP;t.tick();t.stopped(Ramp::INTERRUPTED);}
    {Trial t;t.signal=SIGINT;t.begin();t.stopped(Ramp::INTERRUPTED);}
    {Trial t;t.begin();t.ramp.stop(Ramp::SAFETY_IDLE);t.tick();t.stopped(Ramp::SAFETY_IDLE);}
    for(int input=0;input<2;++input){
        Trial t;t.begin();
        if(input==0)systems::disconnect(t.ramp.input);else systems::disconnect(t.ramp.velocityInput);
        t.tick();t.stopped(Ramp::ERROR);
        if(input==0)systems::connect(t.position.output,t.ramp.input);else systems::connect(t.velocity.output,t.ramp.velocityInput);
        t.tick();t.stopped(Ramp::ERROR);
    }
    {Trial t;t.begin();auto q=t.initial;q[6]=std::numeric_limits<double>::quiet_NaN();t.position.setValue(q);t.tick();t.stopped(Ramp::ERROR);}
    {Trial t;t.begin();Ramp::jv_type v(0.0);v[6]=std::numeric_limits<double>::infinity();t.velocity.setValue(v);t.tick();t.stopped(Ramp::ERROR);}
    std::cout<<"PASS: optional J7 PD hold, captured starting angle, both J8 torque signs, caps, passive/other-axis masking, all-axis displacement/time/watchdog/signal stops, invalid and missing data, no restart.\n";
}

// Runs the actual torque gate in libbarrett's control graph, without CAN.
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/exposed_output.h>
#include "../examples/gravity_ramp_guard.h"

namespace barrett {
double testTime=1;
double highResolutionSystemTime() { return testTime; }
detail::LogFormatter logMessage(const std::string& s,bool output) { return detail::LogFormatter(s,output); }
void detail::LogFormatter::print() { printed=true; }
}
using namespace barrett;
typedef GravityRampGuard Guard;
class Sink : public systems::System, public systems::SingleInput<Guard::jt_type> {
public:
    Guard::jt_type value;
    Sink(systems::ExecutionManager& em) : systems::System("Sink"),systems::SingleInput<Guard::jt_type>(this),value(0.0) { em.startManaging(*this); }
    ~Sink() { mandatoryCleanUp(); }
    void operate() override { value=input.getValue(); }
};
struct Trial {
    systems::ManualExecutionManager em{0.002};
    systems::ExposedOutput<Guard::jp_type> position{Guard::jp_type(0.0)};
    systems::ExposedOutput<Guard::jt_type> gravity{Guard::jt_type(0.0)};
    systems::ExposedOutput<Guard::jt_type> previousSource{Guard::jt_type(123.0)};
    std::atomic<int> signal{0};
    Guard guard;
    Sink sink{em};
    explicit Trial(bool occupied=true,bool extended=false) : guard(&signal,extended) {
        systems::connect(position.output,guard.positionInput);
        systems::connect(gravity.output,guard.gravityInput);
        if(occupied)systems::connect(previousSource.output,sink.input);
        guard.connectOutputTo(sink.input);
        Guard::jt_type g(0.0);
        g[1]=11.876;g[3]=-7.128;g[4]=123.0;g[5]=.531;g[6]=.318;g[7]=.188;
        gravity.setValue(g);
        tick(); guard.arm();
    }
    void tick(double dt=.002,bool refresh=true) {
        testTime+=dt;
        if(refresh) guard.refresh();
        em.runExecutionCycle();
    }
    void begin() { assert(guard.start()); tick(); }
    void latched(Guard::Reason reason) {
        assert(guard.snapshot().reason==reason);
        assert(sink.value.norm()==0.0);
        assert(!guard.start());
        position.setValue(Guard::jp_type(0.0)); gravity.setValue(Guard::jt_type(0.0));
        for(int i=0;i<20;++i) tick();
        assert(guard.snapshot().reason==reason && sink.value.norm()==0.0);
    }
};
int main() {
    // Lost MODE replies are retried only at startup, while this gate is zero.
    {Trial t;for(int i=0;i<650;++i){t.tick(.002,false);assert(t.sink.value.norm()==0);}
     assert(t.guard.snapshot().reason==Guard::ARMED);t.begin();}
    {Trial t(false);assert(t.sink.value.norm()==0);t.begin();}
    {Trial t;bool rejected=false;try{t.guard.connectOutputTo(t.sink.input);}catch(const std::logic_error&){rejected=true;}assert(rejected);}
    // Arbitrary full-model torque cannot escape while waiting for activation.
    {
        Trial t;
        for(int i=0;i<100;++i) t.tick();
        assert(t.sink.value.norm()==0.0);
        t.begin();
        double last=0;bool atCap=false;
        for(int i=0;i<1800 && t.guard.snapshot().reason==Guard::RUNNING;++i) {
            t.tick();const auto s=t.guard.snapshot();
            assert(t.sink.value[4]==0); // passive joint never gets torque
            for(int j=0;j<8;++j) assert(std::fabs(t.sink.value[j])<=t.guard.torqueLimit(j));
            if(s.reason==Guard::RUNNING) {
                assert(s.fraction>=last && s.fraction<=.1);
                assert(std::fabs(t.sink.value[6]-.318*s.fraction)<1e-10);
                assert(std::fabs(t.sink.value[3]-(-7.128)*s.fraction)<1e-10);
                last=s.fraction; atCap|=s.fraction==.1;
            }
        }
        assert(atCap);t.latched(Guard::COMPLETE);
        const auto log=t.guard.samples();
        assert(log.size()>1700 && log.back().reason==Guard::COMPLETE);
        for(double x:log.back().command) assert(x==0);
    }
    // The optional long trial reaches 30% gradually and holds for a full minute.
    // Exercise both the opus 500 Hz rate and the buffer's supported 1 kHz rate.
    for(double dt : {.002,.001}) {
        Trial t(true,true);t.begin();
        double last=0;int held=0;
        while(t.guard.snapshot().reason==Guard::RUNNING) {
            t.tick(dt);const auto s=t.guard.snapshot();
            assert(t.sink.value[4]==0);
            assert(!t.guard.diagnosticLimitsEnabled());
            if(s.reason==Guard::RUNNING) {
                const double expected=.30*std::min(1.0,s.elapsed/10.0);
                assert(s.fraction>=last && s.fraction<=.30);
                assert(std::fabs(s.fraction-expected)<1e-12);
                assert(std::fabs(t.sink.value[1]-11.876*s.fraction)<1e-10);
                assert(std::fabs(t.sink.value[6]-.318*s.fraction)<1e-10);
                if(s.fraction==.30) ++held;
                last=s.fraction;
            }
        }
        assert(held*dt>59.99);
        assert(t.guard.snapshot().elapsed>=70.0 && t.guard.snapshot().elapsed<70.005);
        t.latched(Guard::COMPLETE);
        const auto log=t.guard.samples();
        assert(log.size()>34000 && log.size()<80000 && log.back().reason==Guard::COMPLETE);
        for(double x:log.back().command) assert(x==0);
    }
    // Operator motion on every joint is allowed during the extended trial.
    // The model changes with pose and remains scaled to 30%, with J5 masked.
    for(int joint=0;joint<8;++joint) {
        Trial t(true,true);t.begin();
        for(int i=0;i<5100;++i)t.tick();
        for(int i=1;i<=1000;++i) {
            Guard::jp_type p(0.0);p[joint]=(joint%2?-1:1)*i*.0004;
            Guard::jt_type g(0.0);
            for(int j=0;j<8;++j) g[j]=(j%2?-1:1)*(10+j+std::sin(p[joint]));
            t.position.setValue(p);t.gravity.setValue(g);t.tick();
            assert(t.guard.snapshot().reason==Guard::RUNNING);
            for(int j=0;j<8;++j) assert(std::fabs(t.sink.value[j]-(j==4?0:.30*g[j]))<1e-10);
        }
        const auto s=t.guard.snapshot();
        assert(std::fabs(s.delta[joint])>.39 && std::fabs(s.velocity[joint])>.19);
        assert(std::fabs(t.sink.value[1])>t.guard.torqueLimit(1));
    }
    // Timing, invalid-data, communication/Idle and operator stops remain active
    // late in the long hold; a stopped trial cannot resume.
    for(int fault=0;fault<6;++fault) {
        Trial t(true,true);t.begin();
        for(int i=0;i<20000;++i) t.tick();
        assert(t.guard.snapshot().reason==Guard::RUNNING);
        if(fault==0) {
            systems::disconnect(t.guard.positionInput);t.tick();t.latched(Guard::INVALID_FEEDBACK);
        } else if(fault==1) {
            t.signal=SIGINT;t.tick();t.latched(Guard::INTERRUPTED);
        } else if(fault==2) {
            for(int i=0;i<30;++i)t.tick(.002,false);
            t.latched(Guard::MAIN_TIMEOUT);
        } else if(fault==3) {
            Guard::jt_type g(0.0);g[1]=std::numeric_limits<double>::infinity();
            t.gravity.setValue(g);t.tick();t.latched(Guard::INVALID_FEEDBACK);
        } else if(fault==4) {
            t.guard.stop(Guard::SAFETY_IDLE);t.tick();t.latched(Guard::SAFETY_IDLE);
        } else {
            t.tick(.021);t.latched(Guard::LOOP_DELAY);
        }
    }
    // Motion on every measured joint, including passive J5, cuts all torques.
    for(int joint=0;joint<8;++joint) {
        Trial t;t.begin();
        Guard::jp_type p(0.0);p[joint]=-.021;t.position.setValue(p);t.tick();
        assert(t.guard.snapshot().joint==joint);t.latched(Guard::MOTION);
    }
    // Speed can trip well before the displacement threshold, both directions.
    for(int joint=0;joint<8;++joint) {
        Trial t;t.begin();
        for(int i=1;i<=15 && t.guard.snapshot().reason==Guard::RUNNING;++i) {
            Guard::jp_type p(0.0);p[joint]=(joint%2 ? -1:1)*i*.0005;
            t.position.setValue(p);t.tick();
        }
        assert(std::fabs(t.guard.snapshot().delta[joint])<Guard::motionLimit());
        t.latched(Guard::SPEED);
    }
    // Idle monitoring is already armed; moving during activation cannot rebase.
    { Trial t;Guard::jp_type p(0.0);p[7]=.03;t.position.setValue(p);t.tick();t.latched(Guard::MOTION); }
    // Typical passive encoder jitter must not cause spurious speed faults.
    {
        Trial t;t.begin();
        for(int i=0;i<100;++i) {
            Guard::jp_type p(0.0);p[4]=(i%2)*.00154;t.position.setValue(p);t.tick();
            assert(t.guard.snapshot().reason==Guard::RUNNING);
        }
    }
    // Missing data produces defined zero and stays latched after reconnection.
    {
        Trial t;t.begin();systems::disconnect(t.guard.positionInput);t.tick();
        t.latched(Guard::INVALID_FEEDBACK);
        systems::connect(t.position.output,t.guard.positionInput);t.tick();
        assert(t.sink.value.norm()==0);
    }
    { Trial t;t.begin();systems::disconnect(t.guard.gravityInput);t.tick();t.latched(Guard::INVALID_FEEDBACK); }
    for(int kind=0;kind<2;++kind) {
        Trial t;t.begin();
        if(kind) { Guard::jt_type g(0.0);g[6]=std::numeric_limits<double>::infinity();t.gravity.setValue(g); }
        else { Guard::jp_type p(0.0);p[1]=std::numeric_limits<double>::quiet_NaN();t.position.setValue(p); }
        t.tick();t.latched(Guard::INVALID_FEEDBACK);
    }
    // Real-time cycles continue while main blocks (as with a delayed SDO).
    { Trial t;t.begin();for(int i=0;i<30;++i)t.tick(.002,false);t.latched(Guard::MAIN_TIMEOUT); }
    // A late main-thread refresh cannot conceal a stall.
    { Trial t;t.begin();testTime+=.06;t.guard.refresh();t.tick();t.latched(Guard::MAIN_TIMEOUT); }
    { Trial t;t.begin();t.tick(.021);t.latched(Guard::LOOP_DELAY); }
    { Trial t;t.begin();t.signal=SIGTSTP;t.tick();t.latched(Guard::INTERRUPTED); }
    { Trial t;t.signal=SIGINT;assert(!t.guard.start());t.tick();t.latched(Guard::INTERRUPTED); }
    { Trial t;t.begin();t.guard.stop(Guard::SAFETY_IDLE);t.tick();t.latched(Guard::SAFETY_IDLE); }
    // Corrupt but finite fits must not emit a torque beyond the diagnostic cap.
    { Trial t;t.begin();Guard::jt_type g(100000.0);t.gravity.setValue(g);t.tick();t.latched(Guard::TORQUE_LIMIT); }
    std::cout<<"PASS: real control graph ramp/hold/zero, all-axis motion and speed stops, passive masking, torque caps, invalid/missing inputs, timing, signal and latched stops.\n";
}

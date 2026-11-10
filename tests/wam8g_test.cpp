// Production Wam8g, control graph, kinematics, Puck and CANopen driver.
// Only the CAN transport and OS clock/logging are simulated; no hardware opens.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <barrett/bus/canopen.h>
#include <barrett/systems/wam8g.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/exposed_output.h>
#include "torque_sign_ramp.h"

namespace barrett {
double testTime = 1.0;
double highResolutionSystemTime() { return testTime; }
void btsleepRT(double seconds) { testTime += seconds; }
void btsleep(double seconds) { testTime += seconds; boost::this_thread::interruption_point(); }
void btsleep(double seconds, bool) { btsleep(seconds); }
detail::LogFormatter logMessage(const std::string& s, bool output) { return detail::LogFormatter(s, output); }
std::vector<std::string> printedLog;  // what the library printed for the operator
void detail::LogFormatter::print() {
	if (!printed && ose) { try { printedLog.push_back(str()); } catch (...) {} }
	printed = true;
}
bool wasPrinted(const std::string& text) {
	for (const auto& s : printedLog) if (s.find(text)!=std::string::npos) return true;
	return false;
}
}
using namespace barrett;
using bus::CANopen;
using systems::Wam8g;
struct Frame { int id; size_t len; unsigned char data[8]; };
struct FakeBus : bus::CommunicationsBus {
	typedef std::tuple<int,int,int> Key;
	mutable thread::NullMutex mutex;
	mutable std::deque<Frame> replies;
	mutable std::map<Key,uint32_t> objects;
	mutable std::vector<Frame> sent;
	int32_t counts[8] = {};
	mutable double nextHeartbeat[8] = {};
	int missingPosition = 0;
	mutable int dropBootUpOnce = 0;  // node whose first boot-up frame is lost on the way to the host
	int neverBootsUp = 0;      // node that never sends a boot-up
	FakeBus() {
		objects[Key(10,0x2008,0)] = 1; // IDLE
		for(int n=1;n<=8;++n) {
			objects[Key(n,0x3013,1)] = 4096;
			objects[Key(n,0x6076,0)] = 1000; // one Nm rated
			objects[Key(n,0x6041,0)] = 0x40;
			objects[Key(n,0x1017,0)] = 250;  // heartbeat left on by a previous session
		}
	}
	thread::Mutex& getMutex() const override { return mutex; }
	void open(int) override {} void close() override {} bool isOpen() const override { return true; }
	int receiveRaw(int&, unsigned char*, size_t&, bool) const override { return 1; }
	int send(int id, const unsigned char* data, size_t len) const override {
		Frame f={id,len,{}}; if(len) std::memcpy(f.data,data,len); sent.push_back(f);
		if(id==0 && data[0]==CANopen::NMT_RESET_NODE) {
			// Like a P4 (opus capture 2026-09-22): one last Operational
			// heartbeat a few ms after the command, then the boot-up, and
			// no heartbeats (0x1017 back to 0) until the host configures
			// the producer again.  Every reset puck's boot-up is queued at
			// once, as the real burst arrives long before the later pucks'
			// turn to be awaited.
			int node=data[1];
			replies.push_back(Frame{0x700+node,1,{5}});
			if(node==neverBootsUp) {
			} else if(node==dropBootUpOnce) {
				dropBootUpOnce=0;
			} else {
				replies.push_back(Frame{0x700+node,1,{0}});
			}
			objects[Key(node,0x1017,0)]=0;
			objects[Key(node,0x6060,0)]=0;     // a reset clears the mode setting
			objects[Key(node,0x6041,0)]=0x40;  // and disables the drive
			nextHeartbeat[node-1]=std::numeric_limits<double>::infinity();
		} else if(id==CANopen::COB_SYNC) {
			testTime += 0.002;
			for(int n=1;n<=8;++n) if(n!=missingPosition) {
				Frame p={0x180+n,4,{}};
				for(int b=0;b<4;++b) p.data[b]=(uint32_t(counts[n-1])>>(8*b))&255;
				replies.push_back(p);
			}
		} else if(id>=0x601 && id<=0x60A && len==8) {
			int node=id-0x600, index=data[1]|int(data[2])<<8, sub=data[3];
			Frame reply={0x580+node,8,{}}; std::memcpy(reply.data,data,4);
			if(data[0]==0x40) {
				uint32_t value=index==0x6064 ? counts[node-1] : objects[Key(node,index,sub)];
				reply.data[0]=0x43;
				for(int b=0;b<4;++b) reply.data[4+b]=(value>>(8*b))&255;
			} else {
				int bytes=4-((data[0]>>2)&3); uint32_t value=0;
				for(int b=0;b<bytes;++b) value |= uint32_t(data[4+b])<<(8*b);
				objects[Key(node,index,sub)]=value;
				if(index==0x6040) objects[Key(node,0x6041,0)]=value==15 ? 0x27 : 0x40;
				if(index==0x6060) objects[Key(node,0x6061,0)]=value;
				if(index==0x1017) nextHeartbeat[node-1]=value ? testTime+value/1000.0 : std::numeric_limits<double>::infinity();
				reply.data[0]=0x60;
			}
			replies.push_back(reply);
		}
		return 0;
	}
	int receive(int id, unsigned char* data, size_t& len, bool=false, bool=false) const override {
		for(auto i=replies.begin();i!=replies.end();++i) if(i->id==id) {
			len=i->len; std::memcpy(data,i->data,len); replies.erase(i); return 0;
		}
		if(id>=0x701 && id<=0x708 && testTime>=nextHeartbeat[id-0x701]) {
			nextHeartbeat[id-0x701]=testTime+objects[Key(id-0x700,0x1017,0)]/1000.0;
			len=1; data[0]=5; return 0;
		}
		return 1;
	}
	Frame last(int id) const { for(auto i=sent.rbegin();i!=sent.rend();++i) if(i->id==id) return *i; assert(false); return {}; }
	size_t number(int id) const { size_t n=0; for(const auto& f:sent) if(f.id==id) ++n; return n; }
};
int slot(const Frame& f, int i) { return int16_t(uint16_t(f.data[2*i]) | uint16_t(f.data[2*i+1])<<8); }
template<class F> void throws(F f) { bool caught=false; try { f(); } catch(const std::exception&) { caught=true; } assert(caught); }
void checkNear(int line,double a,double b,double eps=1e-9) {
	if (!(std::fabs(a-b)<eps)) std::cerr << "line " << line << ": " << a << " != " << b << " (tolerance " << eps << ")\n";
	assert(std::fabs(a-b)<eps);
}
#define near(...) checkNear(__LINE__, __VA_ARGS__)

template<class T> class Probe : public systems::System, public systems::SingleInput<T> {
public:
	T value;
	explicit Probe(systems::ExecutionManager& em) : systems::System("Probe"), systems::SingleInput<T>(this) { em.startManaging(*this); }
	~Probe() { this->mandatoryCleanUp(); }
protected:
	void operate() override { value=this->input.getValue(); }
};

void testTorqueRamp() {
    const double savedTime = testTime;
    typedef TorqueSignRamp<8> Ramp;
    for (int scenario=0; scenario<10; ++scenario) {
        systems::ManualExecutionManager em(0.002);
        systems::ExposedOutput<Wam8g::jp_type> position(Wam8g::jp_type(0.0));
        Ramp ramp(5, 4.0, 0.1, 0.02);
        Probe<Wam8g::jt_type> sink(em);
        systems::connect(position.output,ramp.input);
        systems::connect(ramp.output,sink.input);
        em.runExecutionCycle(); near(sink.value.norm(),0.0);
        ramp.start(); testTime += 0.02; ramp.refresh(); em.runExecutionCycle();
        near(sink.value[5],0.8);
        for(int n=0;n<8;++n) if(n!=5) near(sink.value[n],0.0);
        if (scenario==0) {
            Wam8g::jp_type p(0.0); p[5]=0.03; position.setValue(p);
            em.runExecutionCycle(); assert(ramp.getStatus().reason==Ramp::MOTION);
        } else if (scenario==1 || scenario==2) {
            testTime += 0.06;
            if(scenario==2) ramp.refresh(); // late main thread cannot revive it
            em.runExecutionCycle(); assert(ramp.getStatus().reason==Ramp::MAIN_TIMEOUT);
        } else if(scenario==3) {
            for(int i=0;i<31;++i) { testTime+=0.02; ramp.refresh(); em.runExecutionCycle(); }
            assert(ramp.getStatus().reason==Ramp::TIME_LIMIT);
        } else if(scenario==4) {
            ramp.stop(Ramp::SAFETY_IDLE); em.runExecutionCycle();
            assert(ramp.getStatus().reason==Ramp::SAFETY_IDLE);
        } else if(scenario==5) {
            Wam8g::jp_type p(0.0); p[5]=std::numeric_limits<double>::quiet_NaN(); position.setValue(p);
            em.runExecutionCycle(); assert(ramp.getStatus().reason==Ramp::ERROR);
        } else if(scenario==8) {
            Wam8g::jp_type p(0.0); p[1]=std::numeric_limits<double>::quiet_NaN(); position.setValue(p);
            em.runExecutionCycle(); assert(ramp.getStatus().reason==Ramp::ERROR);
        } else {
            const int other = scenario==9 ? 4 : 6;
            Wam8g::jp_type p(0.0); p[other]=0.03;
            if(scenario==7) p[5]=0.03; // both axes: do not declare success
            position.setValue(p); em.runExecutionCycle();
            assert(ramp.getStatus().reason==Ramp::OTHER_MOTION);
            assert(ramp.getStatus().stopJoint==other);
            near(ramp.getStatus().delta[other],0.03);
        }
        near(sink.value.norm(),0.0);
        position.setValue(Wam8g::jp_type(0.0));
        ramp.refresh(); testTime+=0.002; em.runExecutionCycle();
        near(sink.value.norm(),0.0); // stop remains latched
    }
    testTime = savedTime;
    std::cout << "Torque ramp: requested/other-joint motion limits, main-thread stall, safety stop and invalid feedback passed.\n";
}

int main(int argc, char** argv)
{
	assert(argc==2);
	testTorqueRamp();
	libconfig::Config config;
	config.setIncludeDir(argv[1]);
	config.readFile((std::string(argv[1])+"/default.conf").c_str());
	libconfig::Setting& profile=config.lookup("wam8g");
	// The guards are tested against an uncalibrated robot, whatever the
	// shipped template says; the synthetic geometry below sets them.
	profile["kinematics_calibrated"]=false;
	profile["gravity_calibrated"]=false;
	FakeBus bus;
	CANopen::setSensorOnlyNodes(bus,0x20);
	std::vector<std::unique_ptr<Puck> > owned;
	std::vector<Puck*> pucks;
	for(int n=1;n<=8;++n) { owned.emplace_back(new Puck(bus,n)); pucks.push_back(owned.back().get()); }

	{
		// Wake: eight resets in one burst, eight boot-ups queued at once,
		// each preceded by a stale heartbeat and a post-reset Operational
		// heartbeat.  None may be mistaken for, or hide, the boot-up.
		FakeBus wakeBus;
		std::vector<std::unique_ptr<Puck> > wakeOwned;
		std::vector<Puck*> wakePucks;
		for(int n=1;n<=8;++n) { wakeOwned.emplace_back(new Puck(wakeBus,n)); wakePucks.push_back(wakeOwned.back().get()); }
		double t0=testTime;
		Puck::wake(wakePucks);
		for(int n=1;n<=8;++n) {
			assert(wakeBus.objects[FakeBus::Key(n,0x1017,0)]==CANopen::PUCK_HB_PERIOD_MS);  // configured after boot
			for(const auto& f:wakeBus.replies) assert(f.id!=0x700+n);  // every boot-up consumed
		}
		assert(wakeBus.number(0)>=8 && testTime-t0<2.0);  // one boot, not eight timeouts
		// A single lost boot-up frame gets that puck one more reset.
		FakeBus retryBus; retryBus.dropBootUpOnce=7;
		std::vector<std::unique_ptr<Puck> > retryOwned;
		std::vector<Puck*> retryPucks;
		for(int n=1;n<=8;++n) { retryOwned.emplace_back(new Puck(retryBus,n)); retryPucks.push_back(retryOwned.back().get()); }
		Puck::wake(retryPucks);
		size_t resets7=0, resets1=0;
		for(const auto& f:retryBus.sent) if(f.id==0 && f.data[0]==CANopen::NMT_RESET_NODE) { if(f.data[1]==7) ++resets7; if(f.data[1]==1) ++resets1; }
		assert(resets7==2 && resets1==1);
		// A warm wake (the kept-zero start) only reconfigures: no reset, no
		// drive enabled.  A drive that lost its CST arming or is enabled
		// sends the whole set down the reset path instead.
		for (int scenario=0; scenario<3; ++scenario) {
			std::vector<std::unique_ptr<Puck> > warmOwned;
			std::vector<Puck*> warmPucks;
			for(int n=1;n<=8;++n) { warmOwned.emplace_back(new Puck(wakeBus,n)); warmPucks.push_back(warmOwned.back().get()); }
			if (scenario==1) wakeBus.objects[FakeBus::Key(3,0x6060,0)]=0;
			if (scenario==2) wakeBus.objects[FakeBus::Key(6,0x6041,0)]=0x27;
			wakeBus.sent.clear();
			Puck::wake(warmPucks, false);
			size_t resets=0, controlwords=0;
			for(const auto& f:wakeBus.sent) {
				if(f.id==0 && f.data[0]==CANopen::NMT_RESET_NODE) ++resets;
				if(f.id>=0x601 && f.id<=0x608 && f.data[1]==0x40 && f.data[2]==0x60 && f.data[0]!=0x40) ++controlwords;
			}
			assert(scenario==0 ? (resets==0 && controlwords==0) : (resets==8 && controlwords>0));
			for(int n=1;n<=8;++n) assert(warmPucks[n-1]->isAwake());
			Puck::wake(warmPucks, false);  // awake pucks are left alone
		}
		// A puck that never boots up still aborts the wake.
		FakeBus deadBus; deadBus.neverBootsUp=3;
		std::vector<std::unique_ptr<Puck> > deadOwned;
		std::vector<Puck*> deadPucks;
		for(int n=1;n<=8;++n) { deadOwned.emplace_back(new Puck(deadBus,n)); deadPucks.push_back(deadOwned.back().get()); }
		throws([&]{Puck::wake(deadPucks);});
	}

	{
		systems::ManualExecutionManager em(0.002);
		Wam8g wam(&em,pucks,NULL,profile);
		assert(!wam.input.isConnected());  // the generic torque input stays free on a Wam8g
		assert(!wam.hasCalibratedKinematics() && !wam.hasCalibratedGravity());
		Probe<Wam8g::jp_type> measured(em);
		Probe<Wam8g::active_jp_type> activePosition(em);
		Probe<Wam8g::active_jv_type> activeVelocity(em);
		connect(wam.jpOutput,measured.input);
		connect(wam.activeJpOutput,activePosition.input);
		connect(wam.activeJvOutput,activeVelocity.input);
		// A second CAN bus represents a regular WAM7 slave in the same process.
		FakeBus slaveBus;
		Puck slaveJ5(slaveBus,5);
		Puck::wake(std::vector<Puck*>(1,&slaveJ5));
		MotorPuck slaveMotor(&slaveJ5), masterSensor(pucks[4]);
		assert(std::fabs(slaveMotor.getPermillePerNm())>0.0);
		assert(masterSensor.getPermillePerNm()==0.0);
		assert(slaveBus.objects[FakeBus::Key(5,0x6060,0)]==10);
		assert(bus.objects[FakeBus::Key(5,0x6060,0)]!=uint32_t(CANopen::MODE_CST));  // never armed
		assert(CANopen::isSensorOnly(bus,5) && !CANopen::isSensorOnly(slaveBus,5));
		throws([&]{wam.getToolPosition();}); throws([&]{wam.getToolForceInput();});
		throws([&]{wam.getToolTorqueInput();}); throws([&]{wam.gravityCompensate();});
		bus.sent.clear();
		em.runExecutionCycle();
		assert(bus.number(0x80)==1 && bus.number(0x201)==1 && bus.number(0x205)==1);
		assert(bus.sent.front().id==0x80);
		for(int i=0;i<4;++i) { assert(slot(bus.last(0x201),i)==0); assert(slot(bus.last(0x205),i)==0); }
		// All eight samples are consumed, including the passive sensor.
		assert(bus.replies.empty());
		bus.counts[4]=1024; bus.counts[5]=128; bus.counts[6]=256; bus.counts[7]=512;
		em.runExecutionCycle();
		auto q=wam.getJointPositions(); auto active=wam.getActiveJointPositions();
		// puck 6 = J6 roll (N2, reversed), pucks 7 and 8 = the J7/J8 differential (N1):
		// q6 = -m6/N2, q7 = (m7-m8)/(2 N1), q8 = (m7+m8)/(2 N1)   (hand-motion identity check 2026-09-22)
		near(q[4],M_PI/2); near(q[5],-128*2*M_PI/4096/8.7456);
		near(q[6],(256-512)*2*M_PI/4096/(2*7.5077)); near(q[7],(256+512)*2*M_PI/4096/(2*7.5077));
		for(int i=0;i<7;++i) near(active[i],q[i<4?i:i+1]);
		near(measured.value[4],M_PI/2);
		for(int i=0;i<7;++i) {
			near(activePosition.value[i],active[i]);
			near(activeVelocity.value[i],wam.getJointVelocities()[i<4?i:i+1]);
		}
		Wam8g::active_jt_type torque; torque << 0.42,0.5,-0.2,0.18,0.15,-0.06,0.07;
		systems::ExposedOutput<Wam8g::active_jt_type> source(torque);
		connect(source.output,wam.activeInput);
		bus.sent.clear(); em.runExecutionCycle();
		assert(bus.number(0x80)==1 && bus.number(0x201)==1 && bus.number(0x205)==1);
		assert(bus.sent.front().id==0x80);
		assert(slot(bus.last(0x205),0)==0);
		// Independent virtual-work formula for the wrist differential and roll.
		MotorPuck motor(pucks[7]); // use actual driver torque sign/scale
		double scale=motor.getPermillePerNm();
		// tau_m6 = -tau6/N2, tau_m7 = (tau7 + tau8)/(2 N1), tau_m8 = (-tau7 + tau8)/(2 N1)
		// (active torque indices 4,5,6 are J6,J7,J8)
		assert(slot(bus.last(0x205),1)==int(std::lround(-scale*torque[4]/8.7456)));
		assert(slot(bus.last(0x205),2)==int(std::lround(scale*(torque[5]+torque[6])/(2*7.5077))));
		assert(slot(bus.last(0x205),3)==int(std::lround(scale*(-torque[5]+torque[6])/(2*7.5077))));
		for(int i=0;i<7;++i) near(wam.getActiveJointTorques()[i],torque[i]);
		disconnect(wam.activeInput);
		wam.trackReferenceSignal(source.output); em.runExecutionCycle();
		for(int i=0;i<7;++i) near(wam.getActiveJointTorques()[i],torque[i]);
		wam.idle(); em.runExecutionCycle();
		for(int i=0;i<7;++i) near(wam.getActiveJointTorques()[i],0);
		{
			// lbc_torque_sign sees Wam8g through the generic eight-joint API.
			// Its torque reference must preserve the active-input sum and the
			// passive motor mask, and returning the reference to zero must stop it.
			systems::Wam<8>& generic = wam;
			Wam8g::jt_type command(0.0); command[4]=3.0; command[5]=0.3;
			systems::ExposedOutput<Wam8g::jt_type> fullReference(command);
			generic.trackReferenceSignal(fullReference.output);
			em.runExecutionCycle();
			assert(slot(bus.last(0x205),0)==0);
			assert(slot(bus.last(0x205),1)==int(std::lround(-scale*0.3/8.7456)));
			assert(slot(bus.last(0x205),2)==0 && slot(bus.last(0x205),3)==0);
			connect(source.output,wam.activeInput); em.runExecutionCycle();
			for(int i=0;i<7;++i) near(wam.getActiveJointTorques()[i],torque[i]+command[i<4?i:i+1]);
			assert(slot(bus.last(0x205),0)==0);
			assert(slot(bus.last(0x205),1)==int(std::lround(-scale*(torque[4]+0.3)/8.7456)));
			disconnect(wam.activeInput);
			fullReference.setValue(Wam8g::jt_type(0.0)); em.runExecutionCycle();
			for(int i=0;i<4;++i) { assert(slot(bus.last(0x201),i)==0); assert(slot(bus.last(0x205),i)==0); }
			generic.idle();
		}
		{
			// Physical hand-motion identity check: opposite motor rotations
			// belong to J7, equal rotations to J8. Exercise feedback and the
			// corresponding single-joint torque commands through real Wam8g.
			const int previous7 = bus.counts[6], previous8 = bus.counts[7];
			systems::Wam<8>& generic = wam;
			systems::ExposedOutput<Wam8g::jt_type> reference(Wam8g::jt_type(0.0));
			generic.trackReferenceSignal(reference.output);
			for (int axis=0; axis<2; ++axis) {
				bus.counts[6]=previous7; bus.counts[7]=previous8;
				em.runExecutionCycle();
				const auto before = wam.getJointPositions();
				bus.counts[6]+=256;
				bus.counts[7]+=(axis==0 ? -256 : 256);
				em.runExecutionCycle();
				const Wam8g::jp_type delta(wam.getJointPositions()-before);
				for (int j=0; j<8; ++j)
					near(delta[j], j==6+axis ? 256*2*M_PI/4096/7.5077 : 0.0);
				Wam8g::jt_type command(0.0); command[6+axis]=0.2;
				reference.setValue(command); em.runExecutionCycle();
				const int expected = int(std::lround(scale*0.2/(2*7.5077)));
				for (int j=0; j<4; ++j) assert(slot(bus.last(0x201),j)==0);
				assert(slot(bus.last(0x205),0)==0 && slot(bus.last(0x205),1)==0);
				assert(slot(bus.last(0x205),2)==expected);
				assert(slot(bus.last(0x205),3)==(axis==0 ? -expected : expected));
				reference.setValue(Wam8g::jt_type(0.0));
			}
			generic.idle();
			bus.counts[6]=previous7; bus.counts[7]=previous8;
			em.runExecutionCycle();
		}
		// Exercise the exact torque-tool graph through the production Wam8g:
		// fresh encoder motion or a stalled main thread sends zero motor quads.
		for (int scenario=0; scenario<3; ++scenario) {
			TorqueSignRamp<8> ramp(5, 4.0, 4.0, 0.02);
			connect(wam.jpOutput, ramp.input);
			static_cast<systems::Wam<8>&>(wam).trackReferenceSignal(ramp.output);
			em.runExecutionCycle(); ramp.start();
			for (int i=0; i<10; ++i) { ramp.refresh(); em.runExecutionCycle(); }
			assert(slot(bus.last(0x205),1)!=0);
			const int previousCount=bus.counts[5], previous7=bus.counts[6], previous8=bus.counts[7];
			if (scenario==0) bus.counts[5]+=200;
			else if (scenario==1) testTime+=0.08;
			else { bus.counts[6]+=200; bus.counts[7]-=200; } // unintended J7 motion
			em.runExecutionCycle();
			assert(ramp.getStatus().reason==(scenario==0 ? TorqueSignRamp<8>::MOTION :
					(scenario==1 ? TorqueSignRamp<8>::MAIN_TIMEOUT : TorqueSignRamp<8>::OTHER_MOTION)));
			if (scenario==2) assert(ramp.getStatus().stopJoint==6);
			for (int i=0;i<4;++i) { assert(slot(bus.last(0x201),i)==0); assert(slot(bus.last(0x205),i)==0); }
			ramp.refresh(); em.runExecutionCycle();
			assert(slot(bus.last(0x205),1)==0);
			wam.idle(); bus.counts[5]=previousCount; bus.counts[6]=previous7; bus.counts[7]=previous8;
		}
		// The position interface consumes seven values. Moving only J5 changes
		// the full state, without causing the controller to fight that joint.
		em.runExecutionCycle();  // read back the counts restored above first
		systems::ExposedOutput<Wam8g::active_jp_type> position(wam.getActiveJointPositions());
		wam.trackReferenceSignal(position.output); em.runExecutionCycle();
		bus.counts[4]+=500; em.runExecutionCycle();
		assert(slot(bus.last(0x205),0)==0);
		for(int i=0;i<7;++i) near(wam.getActiveJointTorques()[i],0);
		wam.idle();
		{
			// moveTo()/moveHome() connect an eight-wide trajectory; a later
			// seven-wide reference must replace it, not be ignored.
			em.runExecutionCycle();
			systems::ExposedOutput<Wam8g::jp_type> full(wam.getJointPositions());
			static_cast<systems::Wam<8>&>(wam).trackReferenceSignal(full.output);
			em.runExecutionCycle();
			Wam8g::active_jp_type target(wam.getActiveJointPositions()); target[0]+=0.01;
			systems::ExposedOutput<Wam8g::active_jp_type> seven(target);
			wam.trackReferenceSignal(seven.output); em.runExecutionCycle();
			assert(wam.getActiveJointTorques()[0] > 0);
			wam.idle();
		}
		bus.missingPosition=5;
		bus.sent.clear();
		throws([&]{em.runExecutionCycle();}); // sensor loss is not ignored
		assert(bus.number(0x201)==0 && bus.number(0x205)==0);
		bus.missingPosition=0; bus.replies.clear();
	}

	// A synthetic, explicitly test-only serial geometry exercises J5-dependent
	// FK and the seven-column partial Jacobian. This is NOT wrist calibration.
	profile["kinematics_calibrated"]=true;
	// Replace numeric settings so integer-valued template fields become floats.
	for(int i=4;i<8;++i) {
		auto& row=profile["kinematics"]["moving"][i];
		row.remove("alpha_pi"); row.add("alpha_pi",libconfig::Setting::TypeFloat)=0.0;
		row.remove("a"); row.add("a",libconfig::Setting::TypeFloat)=0.0;
	}
	profile["kinematics"]["toolplate"].remove("a");
	profile["kinematics"]["toolplate"].add("a",libconfig::Setting::TypeFloat)=0.0;
	profile["kinematics"]["moving"][4]["alpha_pi"]=0.5;
	profile["kinematics"]["moving"][4]["a"]=0.07;
	profile["kinematics"]["moving"][5]["alpha_pi"]=-0.5;
	profile["kinematics"]["moving"][6]["alpha_pi"]=0.5;
	profile["kinematics"]["toolplate"]["a"]=0.12;
	{
		systems::ManualExecutionManager em(0.002);
		Wam8g wam(&em,pucks,NULL,profile);
		assert(!wam.input.isConnected());  // the generic torque input stays free on a Wam8g
		throws([&]{wam.gravityCompensate();}); // separate gravity calibration gate
		math::Kinematics<8> kin(profile["kinematics"]);
		auto q=wam.getJointPositions(); Wam8g::jv_type zero(0.0);
		kin.eval(q,zero);
		math::Matrix<6,8> full(kin.impl->tool_jacobian);
		auto J=wam.getActiveToolJacobian();
		for(int a=0;a<7;++a) {
			int m=a<4?a:a+1;
			for(int row=0;row<6;++row) near(J(row,a),full(row,m));
			Wam8g::jp_type lo(q),hi(q); lo[m]-=1e-6; hi[m]+=1e-6;
			kin.eval(lo,zero); Wam8g::cp_type plo(kin.impl->tool->origin_pos);
			kin.eval(hi,zero); Wam8g::cp_type phi(kin.impl->tool->origin_pos);
			for(int row=0;row<3;++row) near(J(row,a),(phi[row]-plo[row])/2e-6,1e-7);
		}
		auto before=wam.getToolPosition(); bus.counts[4]+=300; em.runExecutionCycle();
		assert((wam.getToolPosition()-before).norm()>1e-3);
		Wam8g::cf_type force; force << 0.3,-0.1,0.2;
		Wam8g::ct_type moment; moment << 0.02,0.01,-0.01;
		systems::ExposedOutput<Wam8g::cf_type> f(force);
		systems::ExposedOutput<Wam8g::ct_type> t(moment);
		connect(f.output,wam.getToolForceInput()); connect(t.output,wam.getToolTorqueInput());
		em.runExecutionCycle();
		J=wam.getActiveToolJacobian();
		auto jt=wam.getActiveJointTorques();
		for(int a=0;a<7;++a) {
			double expected=0;
			for(int r=0;r<3;++r) expected += J(r,a)*force[r]+J(r+3,a)*moment[r];
			near(jt[a],expected);
		}
		assert(slot(bus.last(0x205),0)==0);
	}

	// Gravity evaluates all eight measured coordinates, then commands only the
	// seven active components. Nonzero synthetic distal mass makes J5 matter.
	profile["gravity_calibrated"]=true;
	profile["gravity_compensation"]["mus"][7][0]=1;
	{
		systems::ManualExecutionManager em(0.002);
		Wam8g wam(&em,pucks,NULL,profile);
		assert(!wam.input.isConnected());  // the generic torque input stays free on a Wam8g
		wam.gravityCompensate(); em.runExecutionCycle();
		assert(wam.isGravityCompensated());
		math::Kinematics<8> kin(profile["kinematics"]);
		bt_calgrav* gc=NULL;
		assert(bt_calgrav_create(&gc,detail::getCSetting(profile["gravity_compensation"]),8)==0);
		units::JointTorques<8>::type full;
		kin.eval(wam.getJointPositions(),Wam8g::jv_type(0.0));
		bt_calgrav_eval(gc,kin.impl,full.asGslType());
		auto before=wam.getActiveJointTorques();
		for(int i=0;i<7;++i) near(before[i],full[i<4?i:i+1]);
		assert(before.norm()>0.1);
		bus.counts[4]+=300; em.runExecutionCycle();
		assert((wam.getActiveJointTorques()-before).norm()>1e-3);
		assert(slot(bus.last(0x205),0)==0);
		wam.gravityCompensate(false); em.runExecutionCycle();
		assert(!wam.isGravityCompensated());
		near(wam.getActiveJointTorques().norm(),0);
		bt_calgrav_destroy(gc);
	}

	// Production software zeroing includes the passive encoder and wrist, and
	// leaves the SB ZERO flag clear (its stock Cartesian model is inapplicable).
	bus.objects[FakeBus::Key(10,0x2000,0)]=208; // safety firmware version
	bus.objects[FakeBus::Key(10,0x2001,0)]=2;   // ROLE_SAFETY
	bus.objects[FakeBus::Key(10,0x2005,0)]=2;   // STATUS_READY
	bus.objects[FakeBus::Key(10,0x202A,0)]=0;   // ZERO
	profile["low_level"]["home"][4]=1.0;  // float: the calibrated home is a float list
	{
		Puck safetyPuck(bus,10);
		SafetyModule safety(&safetyPuck);
		systems::ManualExecutionManager em(0.002);
		Wam8g wam(&em,pucks,&safety,profile);
		Wam8g::jp_type home(profile["low_level"]["home"]);
		auto q=wam.getJointPositions();
		for(int i=0;i<8;++i) near(q[i],home[i],2*M_PI/4096);
		bus.counts[4]+=100; em.runExecutionCycle();
		near(wam.getJointPositions()[4]-q[4],100*2*M_PI/4096);
		assert(bus.objects[FakeBus::Key(10,0x202A,0)]==0);
	}

	// SB r209 keeps that zero across program runs (0x2103): a run that
	// zeroes parks it on the board, the next run continues from it without
	// the home pose, and neither run ever sets the board's ZERO flag.
	bus.objects[FakeBus::Key(10,0x2000,0)]=209;
	for(int n=1;n<=8;++n) bus.objects[FakeBus::Key(n,0x3012,1)]=(700*n)%4096;  // single-turn encoders
	Wam8g::jp_type kept;
	{
		Puck safetyPuck(bus,10);
		SafetyModule safety(&safetyPuck);
		assert(safety.hasZeroRecord() && !safety.wamIsZeroed() && !safety.zeroWasKept());
		systems::ManualExecutionManager em(0.002);
		Wam8g wam(&em,pucks,&safety,profile);           // zeroes at home
		assert(bus.objects[FakeBus::Key(10,0x2103,1)]==0x5A0001FEu);
		kept=wam.getJointPositions();
		safety.setWamZeroed(true);                       // never reaches the board
		assert(bus.objects[FakeBus::Key(10,0x202A,0)]==0);
	}
	// Between the runs motors 2 and 7 turned 300 counts; the pucks kept counting.
	for(int n : {2,7}) {
		bus.counts[n-1]+=300;
		bus.objects[FakeBus::Key(n,0x3012,1)]=(bus.objects[FakeBus::Key(n,0x3012,1)]+300)%4096;
	}
	{
		Puck safetyPuck(bus,10);
		SafetyModule safety(&safetyPuck);
		assert(safety.wamIsZeroed() && safety.zeroWasKept());  // the prompt is skipped
		systems::ManualExecutionManager em(0.002);
		Wam8g wam(&em,pucks,&safety,profile);
		// Same answer as if the first run had watched the motion.
		Wam8g::jp_type motors(0.0);
		motors[1]=-300*2*M_PI/4096;  // canopen_position_sign: -1 on the arm pucks
		motors[6]=300*2*M_PI/4096;
		const Wam8g::jp_type expected(kept+wam.getLowLevelWam().getMotorToJointPositionTransform()*motors);
		for(int i=0;i<8;++i) near(wam.getJointPositions()[i],expected[i],1e-4);
	}
	// A counter that slipped against its encoder (a reset the board missed),
	// or a record lost after the prompt was skipped: stop, never zero blind.
	bus.counts[3]+=700;
	{
		Puck safetyPuck(bus,10);
		SafetyModule safety(&safetyPuck);
		systems::ManualExecutionManager em(0.002);
		throws([&]{Wam8g wam(&em,pucks,&safety,profile);});
	}
	bus.counts[3]-=700;
	{
		Puck safetyPuck(bus,10);
		SafetyModule safety(&safetyPuck);
		assert(safety.wamIsZeroed());
		bus.objects[FakeBus::Key(10,0x2103,1)]=0;       // a puck rebooted meanwhile
		systems::ManualExecutionManager em(0.002);
		throws([&]{Wam8g wam(&em,pucks,&safety,profile);});
		safety.setWamZeroed(false);                      // e.g. zerocal: zero again
		Wam8g wam(&em,pucks,&safety,profile);
		assert(bus.objects[FakeBus::Key(10,0x2103,1)]==0x5A0001FEu);
	}

	// Rough placement at power-up (Barrett's zero compensation): the WAM only
	// sits near home, and each P4's single-turn encoder places the zero.
	{
		// Which way a single-turn encoder counts against its position counter.
		assert(CANopen::singleTurnDirection(300,300)==1);
		assert(CANopen::singleTurnDirection(300,-300)==-1);
		assert(CANopen::singleTurnDirection(-81003,-81003%4096)==1);  // many turns
		assert(CANopen::singleTurnDirection(-81003,81003%4096)==-1);
		assert(CANopen::singleTurnDirection(12,12)==0);         // turned too little to tell
		assert(CANopen::singleTurnDirection(5*4096,0)==0);      // whole turns fit either way
		assert(CANopen::singleTurnDirection(2048,2048)==0);     // so does half a turn
		assert(CANopen::singleTurnDirection(300,900)==0);       // counter not tracking the encoder
		int position=0, singleTurn=0;
		bus.counts[2]=-5000; bus.objects[FakeBus::Key(3,0x3012,1)]=123;
		assert(CANopen::readCounterAndSingleTurn(bus,3,&position,&singleTurn));
		assert(position==-5000 && singleTurn==123);

		bus.objects[FakeBus::Key(10,0x2000,0)]=208;  // no zero record: every run zeroes
		libconfig::Setting& ll=profile["low_level"];
		const Wam8g::jp_type home(ll["home"]);
		const double posSign[8]={-1,-1,-1,-1,1,1,1,1};           // wam8g.conf
		const int trueDir[8]={1,-1,1,-1,1,-1,1,1};               // how each P4 happens to count
		const int measured[8]={1,-1,1,-1,0,-1,1,1};              // zerocal: pinned J5 never turned
		const int atZero[8]={100,3900,2000,5,1234,1500,3000,4095};  // readings with the joints at zero
		Wam8g::jp_type j2mpRow[8];
		// The encoders as they read with the joints at jp
		auto place=[&](const Wam8g::jp_type& jp) {
			for(int i=0;i<8;++i) {
				const double motor=j2mpRow[i].dot(jp);  // this library's motor angle
				const long c=std::lround(atZero[i]+trueDir[i]*posSign[i]*motor*4096/(2*M_PI));
				bus.objects[FakeBus::Key(i+1,0x3012,1)]=((c%4096)+4096)%4096;
			}
		};
		{
			Puck safetyPuck(bus,10);
			SafetyModule safety(&safetyPuck);
			systems::ManualExecutionManager em(0.002);
			Wam8g wam(&em,pucks,&safety,profile);  // no zeroangle yet: zeroed at home
			for(int i=0;i<8;++i) for(int k=0;k<8;++k)
				j2mpRow[i][k]=wam.getLowLevelWam().getJointToMotorPositionTransform()(i,k);
			// zerocal: at its last calibration pose it records each encoder's
			// reading at the joint-zero pose.
			Wam8g::jp_type calPose(0.0); calPose[1]=-1.57; calPose[3]=1.57; calPose[6]=0.21;
			place(calPose);
			libconfig::Setting& za=ll.add("zeroangle",libconfig::Setting::TypeList);
			libconfig::Setting& zd=ll.add("zeroangle_dir",libconfig::Setting::TypeList);
			for(int i=0;i<8;++i) {
				const double reading=bus.objects[FakeBus::Key(i+1,0x3012,1)]*2*M_PI/4096;
				const double z=measured[i] ? wam.getLowLevelWam().singleTurnAtZero(i,reading,calPose,measured[i]) : reading;
				if(measured[i]) near(std::remainder(z-atZero[i]*2*M_PI/4096,2*M_PI),0.0,2e-3);
				za.add(libconfig::Setting::TypeFloat)=z;
				zd.add(libconfig::Setting::TypeFloat)=double(measured[i]);
			}
		}
		// Power-up with the WAM only roughly at home: every motor within half
		// of its half-turn window, except that J4 is 12 deg off in scenario 1
		// (more than half a motor turn) and the file has no directions in 2.
		Wam8g::jp_type rough(0.0);
		const double deg[8]={2,-1,3,5,11,8,6,-5};
		for(int j=0;j<8;++j) rough[j]=deg[j]*M_PI/180;
		for(int scenario=0; scenario<3; ++scenario) {
			Wam8g::jp_type actual(home+rough);
			if(scenario==1) actual[3]+=7*M_PI/180;
			place(actual);
			if(scenario==2) ll.remove("zeroangle_dir");  // an older zerocal file
			printedLog.clear();
			Puck safetyPuck(bus,10);
			SafetyModule safety(&safetyPuck);
			systems::ManualExecutionManager em(0.002);
			Wam8g wam(&em,pucks,&safety,profile);
			const auto q=wam.getJointPositions();
			for(int j=0;j<8;++j) {
				double expected=actual[j];
				if(j==4 || scenario==2) expected=home[j];     // no data: taken to be at home
				if(scenario==1 && j==3) expected-=2*M_PI/18;  // a whole motor turn short
				near(q[j],expected,2e-3);
			}
			assert(wasPrinted("single-turn encoders")==(scenario!=2));
			assert(wasPrinted("J5 --")==(scenario!=2));
			assert(wasPrinted("WARNING: Puck 4")==(scenario==1));
			assert(wasPrinted("WARNING")==(scenario==1));
		}
		ll.remove("zeroangle");
	}

	// Reject attempts to remap or servo the passive joint before touching CAN.
	bus.sent.clear(); profile["joint_position_control"]["kp"][4]=1;
	throws([&]{Wam8g invalid(NULL,pucks,NULL,profile);}); assert(bus.sent.empty());
	profile["joint_position_control"]["kp"][4]=0;
	profile["low_level"]["j2mp"][5][4]=1;
	throws([&]{Wam8g invalid(NULL,pucks,NULL,profile);}); assert(bus.sent.empty());
	profile["low_level"]["j2mp"][5][4]=0;
	profile["low_level"]["j2mp"][4][4]=0.0;
	throws([&]{Wam8g invalid(NULL,pucks,NULL,profile);}); assert(bus.sent.empty());
	profile["low_level"]["j2mp"][4][4]=1.0;
	profile["gravity_compensation"]["mus"].remove(7);
	throws([&]{Wam8g invalid(NULL,pucks,NULL,profile);}); assert(bus.sent.empty());
	std::cout << "PASS wam8g: real eight-node I/O, seven commands, passive isolation, differential torque, FK/Jacobian, wrench/gravity mapping, per-bus masks, eight-joint zeroing and zero keeping, startup guards\n";
}

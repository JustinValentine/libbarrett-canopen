// Production CANopen code, fake bus and clock; no robot or CAN interface.
#include <cassert>
#include <cmath>
#include <limits>
#include <libconfig.h++>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <tuple>
#include <vector>
#include <barrett/os.h>
#include <barrett/bus/canopen.h>

namespace barrett {
double testTime = 0;
double highResolutionSystemTime() { return testTime; }
void btsleepRT(double seconds) { testTime += seconds; }
detail::LogFormatter logMessage(const std::string& s, bool output) { return detail::LogFormatter(s, output); }
void detail::LogFormatter::print() { printed = true; }
}
using barrett::bus::CANopen;
struct DummyMutex : barrett::thread::Mutex {
 void lock() override {} bool try_lock() override { return true; } void unlock() override {}
 int fullUnlock() override { return 0; } void relock(int) override {}
};
struct Frame { int id; size_t len; unsigned char d[8]; };
struct FakeBus : barrett::bus::CommunicationsBus {
 using Key = std::tuple<int,int,int>;
 mutable DummyMutex mutex;
 mutable std::deque<Frame> replies;
 mutable std::map<Key,uint32_t> objects;
 mutable std::vector<Frame> writes;
 mutable bool junk = false, malformed = false, frozenEnabled = false;
 mutable int abortIndex = -1, corruptSub = -1;
 std::function<void(Frame&)> alter;
 FakeBus() {
  objects[Key(10,0x2008,0)] = 1;
  for(int n=1;n<=8;n++) { objects[Key(n,0x3013,1)] = 4096; objects[Key(n,0x6041,0)] = 0x40; }
 }
 barrett::thread::Mutex& getMutex() const override { return mutex; }
 void open(int) override {} void close() override {} bool isOpen() const override { return true; }
 int receiveRaw(int&, unsigned char*, size_t&, bool) const override { return 1; }
 int send(int id, const unsigned char* data, size_t len) const override {
  Frame f={id,len,{}}; std::memcpy(f.d,data,len); writes.push_back(f);
  if(id<0x601 || id>0x60A || len!=8) return 0;
  int node=id-0x600, index=data[1]|int(data[2])<<8, sub=data[3];
  Frame reply={0x580+node,8,{}}; std::memcpy(reply.d,data,4);
  bool upload=data[0]==0x40;
  if(index==abortIndex) { reply.d[0]=0x80; reply.d[7]=0x08; }
  else if(upload) {
   uint32_t value=objects[Key(node,index,sub)];
   reply.d[0]=0x43;
   if(index==0x2100 && sub==corruptSub) value ^= 2;
   for(int i=0;i<4;i++) reply.d[4+i]=(value>>(8*i))&0xFF;
  } else {
   int bytes=4-((data[0]>>2)&3); uint32_t value=0;
   for(int i=0;i<bytes;i++) value |= uint32_t(data[4+i])<<(8*i);
   objects[Key(node,index,sub)]=value;
   if(index==0x6040) {
    if(value==15) objects[Key(node,0x6041,0)]=0x27;
    else if(value==0 && !frozenEnabled) objects[Key(node,0x6041,0)]=0x40;
   }
   if(index==0x6060) objects[Key(node,0x6061,0)]=value;
   reply.d[0]=0x60;
  }
  if(alter) alter(reply);
  replies.push_back(reply); return 0;
 }
 int receive(int, unsigned char* data, size_t& len, bool=false, bool=false) const override {
  barrett::testTime += 0.0001;
  if(junk || malformed) {
   std::memset(data,0,8); data[0]=0x43; data[1]=0xFF; len=malformed?2:8; return 0;
  }
  if(replies.empty()) return 1;
  Frame f=replies.front(); replies.pop_front(); len=f.len; std::memcpy(data,f.d,len); return 0;
 }
};

// Only robot construction is stubbed here. These ProductManager methods are
// extracted verbatim by the runner and use real libconfig and CANopen code.
namespace barrett {
struct Puck {
 enum { PT_Motor, PT_Safety, PT_Unknown, MODE=8, ZERO=42 };  // r197 safety-role numbering
 int id, type;
 const bus::CommunicationsBus& bus;
 static int wakes, sbVers;
 static bool failWake, lastWakeReset;
 Puck(int n, int t, const bus::CommunicationsBus& b) : id(n), type(t), bus(b) {}
 int getId() const { return id; } int getType() const { return type; }
 int getVers() const { return type==PT_Safety ? sbVers : 200; }
 void setProperty(int prop, int value, bool) { bus::CANopen::sdoDownload(bus,id,0x2000+prop,0,value,2); }
 static void wake(const std::vector<Puck*>&, bool resetFirst) { ++wakes; lastWakeReset=resetFirst; if(failWake) throw std::runtime_error("wake failed"); }
};
int Puck::wakes=0, Puck::sbVers=208; bool Puck::failWake=false, Puck::lastWakeReset=false;
struct SafetyModule { enum { ESTOP=0 }; };
namespace systems { struct RealTimeExecutionManager { RealTimeExecutionManager(double,int) {} }; }
struct ProductManager {
 enum { MAX_WAM_DOF=8, SAFETY_MODULE_ID=10 };
 libconfig::Config config;
 bus::CommunicationsBus* bus;
 std::vector<Puck*> pucks, wamPucks;
 systems::RealTimeExecutionManager* rtem = nullptr;
 explicit ProductManager(FakeBus& b, const char* conf = "bus:{expected_puck_mask=0x1FE;};wam8g:{};") : bus(&b), wamPucks(8) {
  config.readString(conf);
  for(int n=1;n<=8;n++) { pucks.push_back(new Puck(n,Puck::PT_Motor,b)); if(n<=8) wamPucks[n-1]=pucks.back(); }
  pucks.push_back(new Puck(10,Puck::PT_Safety,b)); Puck::wakes=0; Puck::failWake=false; Puck::sbVers=208;
 }
 ~ProductManager() { for(auto p:pucks) delete p; delete rtem; }
 const std::vector<Puck*>& getPucks() const { return pucks; }
 Puck* getPuck(int) const;
 void remove(int id) { for(auto i=pucks.begin();i!=pucks.end();++i) if((*i)->id==id) {delete *i;pucks.erase(i);break;} if(id<=8) wamPucks[id-1]=nullptr; }
 int expectedPuckMask() const; int presentPuckMask() const; bool expectedPucksPresent() const;
 bool foundSafetyModule() const;
 bool foundWam3() const; bool foundWam4() const; bool foundWam7() const; bool foundWam8g() const;
 bool verifyWamPucks(size_t) const; void wakeAllPucks() const;
 systems::RealTimeExecutionManager* getExecutionManager(double,int);
};
#include "product_manager_under_test.inc"
}

template<class F> void throws(F f) { bool caught=false; try { f(); } catch(const std::exception&) { caught=true; } assert(caught); }
// A zero record as LowLevelWam leaves it on an r209 board (key, then per node
// joint position in micro-radians, raw 0x6064, single-turn 0x3012:01), and the
// pucks as they read now: counted[n] counts further, turned[n] on the encoder.
void storeZero(FakeBus& bus, int mask, const std::map<int,int>& counted = {}, const std::map<int,int>& turned = {}) {
 bus.objects[FakeBus::Key(10,0x2103,1)] = 0x5A000000u | mask;
 for(int n=1;n<=8;n++) if(mask & (1<<n)) {
  const int raw0 = 100*n - 350, mech0 = (400*n + 3000) % 4096;
  bus.objects[FakeBus::Key(10,0x2103,1+n)] = uint32_t(int32_t(-250000*n));
  bus.objects[FakeBus::Key(10,0x2103,9+n)] = uint32_t(raw0);
  bus.objects[FakeBus::Key(10,0x2103,17+n)] = uint32_t(mech0);
  const int c = counted.count(n) ? counted.at(n) : 0, t = counted.count(n) && !turned.count(n) ? c : (turned.count(n) ? turned.at(n) : 0);
  bus.objects[FakeBus::Key(n,0x1017,0)] = 250;
  bus.objects[FakeBus::Key(n,0x6064,0)] = uint32_t(raw0 + c);
  bus.objects[FakeBus::Key(n,0x3012,1)] = uint32_t(((mech0 + t) % 4096 + 4096) % 4096);
 }
}
int main() {
 int cases=0;
 for(bool malformed : {false,true}) {
  FakeBus bus; bus.junk=!malformed; bus.malformed=malformed; int value=0;
  double start=barrett::testTime;
  assert(CANopen::sdoUpload(bus,1,0x1000,0,&value,0.01)==1);
  assert(barrett::testTime-start<0.011);
  throws([&]{ CANopen::sdoDownload(bus,1,0x6040,0,0,2); });
  ++cases;
 }
 for(int bytes=1;bytes<=4;bytes++) {
  for(uint32_t value : {uint32_t(0),uint32_t(0x7F),uint32_t(0x80),UINT32_MAX,uint32_t(0x80000000)}) {
   FakeBus bus; bus.alter=[=](Frame& f){ f.d[0]=0x43|((4-bytes)<<2); for(int i=0;i<4;i++) f.d[4+i]=value>>(8*i); };
   int result=0; assert(CANopen::sdoUpload(bus,1,0x6064,0,&result)==0);
   uint32_t truncated=value & (UINT32_MAX >> (8*(4-bytes)));
   int64_t expected=truncated;
   if(truncated & (uint32_t(1)<<(8*bytes-1))) expected -= int64_t(1)<<(8*bytes);
   assert(result==expected);
  }
  ++cases;
 }
 for(int cs : {0x41,0x53,0x46}) {
  FakeBus bus; bus.alter=[=](Frame& f){ f.d[0]=cs; }; int value=0;
  throws([&]{ CANopen::sdoUpload(bus,1,0x1000,0,&value); }); ++cases;
 }
 {
  FakeBus bus; CANopen::configureSafetyNodes(bus,0x1FE,0x20);
  assert(bus.objects[FakeBus::Key(10,0x2100,4)]==0x20);
  assert(bus.objects[FakeBus::Key(10,0x2100,5)]==0x1FE); ++cases;
 }
 for(int sub : {4,5}) {
  FakeBus bus; bus.corruptSub=sub;
  throws([&]{ CANopen::configureSafetyNodes(bus,0x1FE,0x20); }); ++cases;
 }
 {
  FakeBus bus; bus.abortIndex=0x2100;
  throws([&]{ CANopen::configureSafetyNodes(bus,0x1FE,0x20); }); ++cases;
 }
 for(int mode : {0,2}) {
  FakeBus bus; bus.objects[FakeBus::Key(10,0x2008,0)]=mode;
  throws([&]{ CANopen::configureSafetyNodes(bus,0x1FE,0x20); });
  assert(bus.writes.size()==1 && bus.writes.front().d[0]==0x40); ++cases;
 }
 for(int counts : {0,2048,4096,8192}) {
  FakeBus bus; bus.objects[FakeBus::Key(3,0x3013,1)]=counts;
  if(counts==4096) CANopen::validateWamEncoders(bus);
  else throws([&]{ CANopen::validateWamEncoders(bus); }); ++cases;
 }
 {
  FakeBus bus; bus.objects[FakeBus::Key(1,0x3013,1)]=8192;
  throws([&]{ CANopen::configureMotorPuck(bus,1); });
  assert(bus.writes.size()==1 && bus.writes.front().d[0]==0x40); ++cases;
 }
 for(int index : {0x1006,0x60C2}) {
  FakeBus bus; bus.abortIndex=index;
  throws([&]{ CANopen::configureMotorPuck(bus,1); }); ++cases;
 }
 {
  FakeBus bus; CANopen::armCstMode(bus,1);
  assert(bus.objects[FakeBus::Key(1,0x6041,0)]==0x40);
  assert(bus.writes.front().d[1]==0x71 && bus.writes.front().d[2]==0x60 && bus.writes.front().d[4]==0); ++cases;
 }
 for(bool failedMode : {false,true}) {
  FakeBus bus; bus.frozenEnabled=!failedMode; bus.abortIndex=failedMode?0x6060:-1;
  throws([&]{ CANopen::armCstMode(bus,1); });
  assert(bus.objects[FakeBus::Key(1,0x6040,0)]==0);
  assert(bus.objects[FakeBus::Key(10,0x2008,0)]==0); ++cases;
 }
 for(int missing=1;missing<=8;missing++) {
  FakeBus bus; barrett::ProductManager pm(bus); pm.remove(missing);
  assert(!pm.foundWam3() && !pm.foundWam4() && !pm.foundWam7() && !pm.foundWam8g());
  throws([&]{pm.wakeAllPucks();}); assert(bus.writes.empty() && barrett::Puck::wakes==0); ++cases;
 }
 {
  FakeBus bus; barrett::ProductManager pm(bus);
  assert(pm.foundWam8g() && !pm.foundWam4() && !pm.foundWam7());
  bus.objects[FakeBus::Key(10,0x202A,0)] = 1;  // stale ZERO flag on the SB
  pm.wakeAllPucks(); assert(barrett::Puck::wakes==1 && CANopen::isSensorOnly(bus,5));
  assert(bus.objects[FakeBus::Key(10,0x202A,0)]==0);  // wake's reset voids zeroing: flag cleared
  ++cases;
 }
 {
  // No expected mask: legacy behaviour, the product is whatever answered.
  // Eight motor-typed pucks without a wam8g profile are
  // not any WAM, but waking them is still allowed (the SB watches the
  // detected set, no sensor-only node).
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{};");
  assert(pm.expectedPuckMask()==0 && pm.expectedPucksPresent());
  assert(!pm.foundWam4() && !pm.foundWam7() && !pm.foundWam8g());
  pm.wakeAllPucks();
  assert(bus.objects[FakeBus::Key(10,0x2100,5)]==0x1FE && bus.objects[FakeBus::Key(10,0x2100,4)]==0);
  assert(CANopen::getSensorOnlyNodes(bus)==0); ++cases;
 }
 {
  // No expected mask, a plain 4-DOF WAM: detected and woken as before.
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{};");
  for(int node=5;node<=8;++node) pm.remove(node);
  assert(pm.foundWam4() && !pm.foundWam7());
  pm.wakeAllPucks(); assert(bus.objects[FakeBus::Key(10,0x2100,5)]==0x1E); ++cases;
 }
 {
  // 0x01FE declared without a wam8g profile is refused.
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{expected_puck_mask=0x1FE;};");
  throws([&]{pm.foundWam4();}); throws([&]{pm.wakeAllPucks();});
  assert(bus.writes.empty()); ++cases;
 }
  for(int mask : {0,-1,1,0x200}) {
  FakeBus bus; barrett::ProductManager pm(bus); pm.config.lookup("bus.expected_puck_mask")=mask;
  throws([&]{pm.wakeAllPucks();}); assert(bus.writes.empty()); ++cases;
 }
 for(int sub : {4,5}) {
  FakeBus bus; barrett::ProductManager pm(bus); bus.corruptSub=sub;
  throws([&]{pm.wakeAllPucks();}); assert(barrett::Puck::wakes==0); ++cases;
 }
 {
  FakeBus bus; barrett::ProductManager pm(bus); pm.getPuck(8)->type=barrett::Puck::PT_Unknown;
  assert(!pm.foundWam4() && !pm.foundWam7()); ++cases;
 }
 {
  FakeBus bus; barrett::ProductManager pm(bus); barrett::Puck::failWake=true;
  throws([&]{pm.wakeAllPucks();}); assert(bus.objects[FakeBus::Key(10,0x2008,0)]==0); ++cases;
 }
 for(double period : {0.0,0.001,0.002,0.004,std::numeric_limits<double>::quiet_NaN()}) {
  FakeBus bus; barrett::ProductManager pm(bus);
  if(period==0.002) assert(pm.getExecutionManager(period,50)!=nullptr);
  else throws([&]{pm.getExecutionManager(period,50);}); ++cases;
 }
 // Both included profiles must select ONE combined product, never arm+wrist.
 for(int missing=0; missing<=8; ++missing) {
  FakeBus bus; barrett::ProductManager pm(bus);
  if(missing) pm.remove(missing);
  assert(pm.foundWam8g()==(missing==0));
  assert(!pm.foundWam3() && !pm.foundWam4() && !pm.foundWam7());
  if(missing) { throws([&]{pm.wakeAllPucks();}); assert(bus.writes.empty()); }
  else {
   pm.wakeAllPucks();
   assert(CANopen::getSensorOnlyNodes(bus)==0x20);
   assert(bus.objects[FakeBus::Key(10,0x2100,4)]==0x20);
   assert(bus.objects[FakeBus::Key(10,0x2100,5)]==0x1FE);
  }
  ++cases;
 }
 for(int dof : {3,4,7}) {
  FakeBus bus; barrett::ProductManager pm(bus);
  pm.config.lookup("bus.expected_puck_mask")=(1<<(dof+1))-2;
  for(int node=dof+1;node<=8;++node) pm.remove(node);
  assert(!pm.foundWam8g());
  assert(pm.foundWam3()==(dof==3) && pm.foundWam4()==(dof==4) && pm.foundWam7()==(dof==7));
  pm.wakeAllPucks(); assert(CANopen::getSensorOnlyNodes(bus)==0);
  ++cases;
 }
 {
  FakeBus master, slave;
  barrett::ProductManager mp(master), sp(slave);
  sp.config.lookup("bus.expected_puck_mask")=0xFE; sp.remove(8);
  mp.wakeAllPucks(); sp.wakeAllPucks();
  assert(CANopen::getSensorOnlyNodes(master)==0x20);
  assert(CANopen::getSensorOnlyNodes(slave)==0);
  assert(CANopen::isSensorOnly(master,5) && !CANopen::isSensorOnly(slave,5));
  mp.wakeAllPucks();
  assert(CANopen::getSensorOnlyNodes(slave)==0);
  ++cases;
 }
 // Keeping the WAM zero across runs (SB r209, 0x2103).  A 4-DOF WAM.
 auto wam4 = [](FakeBus& bus, barrett::ProductManager& pm) {
  for(int node=5;node<=8;++node) pm.remove(node);
  barrett::Puck::sbVers = 209; (void)bus;
 };
 const uint32_t key4 = 0x5A00001Eu;
 {
  // Pucks kept counting (joint 2 moved 1000 counts, its encoder with it,
  // joint 3 the other way round): no resets, the record stays.
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{};"); wam4(bus,pm);
  storeZero(bus,0x1E,{{2,1000},{3,-4000}},{{3,4000}});
  pm.wakeAllPucks();
  assert(barrett::Puck::wakes==1 && !barrett::Puck::lastWakeReset);
  assert(bus.objects[FakeBus::Key(10,0x2103,1)]==key4); ++cases;
 }
 {
  // Also kept on r208+ only: an older board has no record, pucks are reset.
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{};"); wam4(bus,pm); barrett::Puck::sbVers = 208;
  storeZero(bus,0x1E);
  pm.wakeAllPucks(); assert(barrett::Puck::lastWakeReset); ++cases;
 }
 {
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{};"); wam4(bus,pm);   // nothing stored
  pm.wakeAllPucks(); assert(barrett::Puck::wakes==1 && barrett::Puck::lastWakeReset); ++cases;
 }
 for(int scenario=0; scenario<4; ++scenario) {
  // A puck that was reset behind the board's back, a counter that slipped
  // against its encoder, a moving/unanswering puck or another puck set: the
  // record is cleared and the pucks are reset, so the operator homes the WAM.
  FakeBus bus; barrett::ProductManager pm(bus,"bus:{};"); wam4(bus,pm);
  if(scenario==0) { storeZero(bus,0x1E); bus.objects[FakeBus::Key(3,0x1017,0)]=0; }
  if(scenario==1) storeZero(bus,0x1E,{{4,700}},{{4,100}});
  if(scenario==2) storeZero(bus,0x1E,{{4,4096*3+40}},{{4,0}});
  if(scenario==3) storeZero(bus,0x1FE);
  pm.wakeAllPucks();
  assert(barrett::Puck::wakes==1 && barrett::Puck::lastWakeReset);
  assert(bus.objects[FakeBus::Key(10,0x2103,1)]==0); ++cases;
 }
 std::cout<<"PASS "<<cases<<" CANopen/startup scenarios (production transport and extracted ProductManager methods)\n";
}

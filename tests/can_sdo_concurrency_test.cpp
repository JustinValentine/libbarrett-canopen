// Real BusManager, CANopen and Puck code over an in-memory CAN transport.
// No sockets are opened. Wall clock and mutexes are real.
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstring>
#include <deque>
#include <future>
#include <iostream>
#include <thread>
#include <barrett/os.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/canopen.h>
#include <barrett/products/puck.h>

namespace barrett {
double highResolutionSystemTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void btsleepRT(double seconds) { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); }
detail::LogFormatter logMessage(const std::string& s, bool out) { return detail::LogFormatter(s, out); }
void detail::LogFormatter::print() { printed = true; }
namespace bus {
// BusManager's unused owning constructors must link, but must never open CAN.
CANSocket::CANSocket() { throw std::logic_error("Hardware forbidden in this test"); }
CANSocket::CANSocket(int) { throw std::logic_error("Hardware forbidden in this test"); }
CANSocket::~CANSocket() {}
void CANSocket::open(int) { throw std::logic_error("Hardware forbidden in this test"); }
void CANSocket::close() {}
bool CANSocket::isOpen() const { return false; }
int CANSocket::send(int, const unsigned char*, size_t) const { return 2; }
int CANSocket::receiveRaw(int&, unsigned char*, size_t&, bool) const { return 2; }
}
}
using namespace barrett;
using bus::CANopen;
struct Frame { int id; size_t len; unsigned char data[8]; };
struct Transport : bus::CommunicationsBus {
    mutable thread::RealTimeMutex io;
    mutable std::deque<Frame> frames, delayed;
    mutable std::atomic<int> requests{0}, syncs{0}, quads{0}, heartbeats{0};
    std::atomic<bool> release{false}, throwRead{false};
    int delayIndex = -1, dropIndex = -1;
    thread::Mutex& getMutex() const override { return io; }
    void open(int) override {} void close() override {} bool isOpen() const override { return true; }
    int send(int id, const unsigned char* data, size_t len) const override {
        BARRETT_SCOPED_LOCK(io);
        if (id == CANopen::COB_SYNC) {
            ++syncs;
            for (int n = 1; n <= 8; ++n) frames.push_back(Frame{0x180+n, 4, {static_cast<unsigned char>(n)}});
        } else if (id == CANopen::COB_QUAD_A || id == CANopen::COB_QUAD_B) ++quads;
        else if (id == CANopen::COB_HB_HOST) ++heartbeats;
        else if (id >= 0x601 && id <= 0x67f && len == 8) {
            ++requests;
            int index = data[1] | (int(data[2]) << 8);
            if (index == dropIndex) return 0;
            Frame reply{id - 0x80, 8, {}};
            std::memcpy(reply.data, data, 4);
            reply.data[0] = data[0] == 0x40 ? 0x43 : 0x60;
            reply.data[4] = data[1]; reply.data[5] = data[2];
            (index == delayIndex ? delayed : frames).push_back(reply);
        }
        return 0;
    }
    int receiveRaw(int& id, unsigned char* data, size_t& len, bool) const override {
        BARRETT_SCOPED_LOCK(io);
        if (throwRead) throw std::runtime_error("injected transport failure");
        if (release && !delayed.empty()) { frames.push_back(delayed.front()); delayed.pop_front(); }
        if (frames.empty()) return 1;
        Frame f = frames.front(); frames.pop_front();
        id=f.id; len=f.len; std::memcpy(data,f.data,len); return 0;
    }
};
void awaitRequests(Transport& t, int count) {
    const double deadline = highResolutionSystemTime() + 2;
    while (t.requests < count && highResolutionSystemTime() < deadline) btsleepRT(0.001);
    assert(t.requests >= count);
}
struct Controller {
    Transport& transport;
    bus::BusManager& manager;
    std::atomic<bool> stop{false};
    std::thread worker;
    double maxGap = 0;
    Controller(Transport& t, bus::BusManager& b) : transport(t), manager(b), worker([this] {
        double previous = highResolutionSystemTime();
        while (!stop) {
            {
                BARRETT_SCOPED_LOCK(manager.getMutex());
                const double now = highResolutionSystemTime();
                maxGap = std::max(maxGap, now - previous); previous = now;
                CANopen::sync(manager);
                unsigned char data[8] = {}; size_t len;
                for (int n=1; n<=8; ++n) {
                    assert(manager.receive(0x180+n,data,len,true,true)==0);
                    assert(len==4 && data[0]==n);
                }
                manager.send(CANopen::COB_QUAD_A,data,8);
                manager.send(CANopen::COB_QUAD_B,data,8);
                if (transport.syncs % 10 == 0) CANopen::hostHeartbeat(manager);
            }
            btsleepRT(0.002);
        }
    }) {}
    void finish() { stop=true; if (worker.joinable()) worker.join(); }
    ~Controller() { finish(); }
};
template<class F> void lostReply(const char* name, F f) {
    Transport transport; bus::BusManager bus(&transport);
    transport.dropIndex=0x2008;
    {
        Controller controller(transport,bus);
        auto request=std::async(std::launch::async,[&] { f(bus); });
        awaitRequests(transport,1);
        const int before=transport.syncs;
        btsleepRT(0.12);
        assert(transport.syncs-before >= 20);
        assert(request.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
        request.get();
        controller.finish();
        assert(controller.maxGap < 0.1);
        assert(transport.quads == 2*transport.syncs && transport.heartbeats > 0);
        std::cout << name << ": " << transport.syncs << " cycles, max gap " << controller.maxGap*1000 << " ms\n";
    }
    // The timed-out transaction must release ownership too.
    auto next=std::async(std::launch::async,[&] { int v=0; assert(CANopen::sdoUpload(bus,10,0x2009,0,&v)==0); assert(v==0x2009); });
    assert(next.wait_for(std::chrono::seconds(2))==std::future_status::ready); next.get();
}
int main() {
    lostReply("Puck MODE timeout",[](const bus::CommunicationsBus& b) {
        bool caught=false; try { Puck::getProperty(b,10,8); } catch(const std::runtime_error&) { caught=true; } assert(caught);
    });
    lostReply("Puck tryGetProperty wait",[](const bus::CommunicationsBus& b) {
        int v=0; assert(Puck::tryGetProperty(b,10,8,&v,0.2)==1);
    });
    lostReply("CANopen upload timeout",[](const bus::CommunicationsBus& b) {
        int v=0; assert(CANopen::sdoUpload(b,10,0x2008,0,&v,0.2)==1);
    });
    lostReply("CANopen download timeout",[](const bus::CommunicationsBus& b) {
        bool caught=false; try { CANopen::sdoDownload(b,10,0x2008,0,1,2); } catch(const std::runtime_error&) { caught=true; } assert(caught);
    });
    Transport t; bus::BusManager bus(&t);
    assert(&bus.getSdoMutex()==&t.getSdoMutex());
    assert(&bus.getSdoMutex()!=&bus.getMutex());
    t.delayIndex=0x2008;
    auto first=std::async(std::launch::async,[&] { return Puck::getProperty(bus,10,8); });
    awaitRequests(t,1);
    // Cross-API clients to the same node cannot steal each other's reply.
    auto second=std::async(std::launch::async,[&] { int v=0; assert(CANopen::sdoUpload(bus,10,0x2009,0,&v)==0); return v; });
    btsleepRT(0.12); assert(t.requests==1);
    // An unrelated bus must not share the transaction lock.
    Transport other; bus::BusManager otherBus(&other);
    assert(Puck::getProperty(otherBus,10,9)==0x2009);
    t.release=true;
    assert(first.get()==0x2008); assert(second.get()==0x2009);
    t.throwRead=true;
    bool caught=false;
    try { Puck::getProperty(bus,10,9); } catch(const std::runtime_error&) { caught=true; }
    assert(caught); t.throwRead=false;
    auto after=std::async(std::launch::async,[&] { return Puck::getProperty(bus,10,9); });
    assert(after.wait_for(std::chrono::seconds(2))==std::future_status::ready); assert(after.get()==0x2009);
    std::cout << "SDO ownership, bus isolation, timeout and exception cleanup passed.\n";
}

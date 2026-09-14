// File: tests/test_protocol.cpp
// Golden wire frames and failure scenarios for the real generated Arduino runtime.
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include "kinisi.h"
#include "kinisi_codec.h"
TwoWire Wire;
MockSerial Serial;
uint32_t fake_millis=0;

/** Compose a fixture frame independently of the production frame builder. */
static std::vector<uint8_t> frame(uint8_t cmd, uint16_t id, std::vector<uint8_t> payload={}) {
    std::vector<uint8_t> data={uint8_t(payload.size()+3), cmd, uint8_t(id), uint8_t(id>>8)};
    data.insert(data.end(),payload.begin(),payload.end());
    return data;
}
/** Reply to INIT with board identity and its final uptime READY response. */
static void identity(const std::vector<uint8_t>& request) {
    assert(request.size()==12 && request[0]==11 && request[1]==0x70);
    const std::vector<uint8_t> expected={3,2,0,0,2,0,0,0};
    assert(std::vector<uint8_t>(request.begin()+4,request.end())==expected);
    uint16_t id=request[2] | uint16_t(request[3])<<8;
    Wire.responses.push_back(frame(0x70,id,{1,0,3,1,2,0,0,0x78,0x56,0x34,0x12,0xef,0xcd,0xab,0x90}));
    Wire.responses.push_back(frame(0x73,id,{0}));
}
/** Reset fake transport and create an INIT/ACK responder. */
static void reset() {
    Wire=TwoWire(); fake_millis=0;
    Wire.responder=[](const std::vector<uint8_t>& r) {
        assert(r.size()==size_t(r[0])+1);
        if(r[1]==0x70) identity(r);
        else Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3])<<8));
    };
}
/** Check readiness, identity and golden multi-byte request encoding. */
static void handshake_and_encoding() {
    reset(); KinisiController c;
    assert(!c.stop_motor(0));
    assert(c.lastError().failure==KinisiFailure::NOT_READY && Wire.transactions==0);
    assert(c.begin() && c.ready() && Wire.bus_timeout==100000);
    assert(c.boardInfo().board_patch==1 && c.boardInfo().protocol_major==2);
    assert(c.boardInfo().firmware_build_high==0x12345678 && c.boardInfo().firmware_build_low==0x90abcdef);
    assert(c.set_controller_frequency(1000));
    assert(Wire.sent==std::vector<uint8_t>({5,0x0a,2,0,0xe8,3}));
    assert(c.set_motor_speed(2,-12.5));
    assert(Wire.sent==std::vector<uint8_t>({12,2,3,0,2,0,0,0,0,0,0,0x29,0xc0}));
    assert(c.lastError().failure==KinisiFailure::NONE);
}
/** Decode a 64-bit acquisition timestamp and binary64 angle, without native struct casts. */
static void odometry_and_error() {
    reset(); KinisiController c; assert(c.begin());
    Wire.responder=[](const std::vector<uint8_t>& r) {
        uint16_t id=r[2] | uint16_t(r[3])<<8;
        if(r[1]==0x16) Wire.responses.push_back(frame(0x16,id,{0x08,7,6,5,4,3,2,1,0,1,0,0,0,0,0,0,0xf4,0x3f}));
        else Wire.responses.push_back(frame(0x7f,id,{r[1],9}));
    };
    encoder_odometry_sample sample=c.get_encoder_odometry(0);
    assert(c.lastError().failure==KinisiFailure::NONE);
    assert(sample.timestamp_us==UINT64_C(0x0102030405060708) && sample.angle==1.25);
    assert(sample.clock_mode==0 && sample.clock_quality==1);
    assert(!c.start_encoder_odometry(0));
    assert(c.lastError().failure==KinisiFailure::CONTROLLER);
    assert(c.lastError().code==KinisiErrorCode::ODOMETRY_NOT_INITIALIZED);
    assert(c.lastError().command==0x13 && c.lastError().message_id==3 && c.ready());
    sample=c.get_encoder_odometry(0);
    assert(sample.angle==1.25 && c.lastError().failure==KinisiFailure::NONE);
}
/** Exercise wraparound, mismatched IDs and a matching ID with the wrong command. */
static void correlation() {
    reset(); KinisiController c; assert(c.begin());
    for(unsigned i=2;i<=65535;++i) {
        assert(c.toggle_status_led_state());
        assert((Wire.sent[2] | unsigned(Wire.sent[3])<<8)==i);
    }
    assert(c.toggle_status_led_state() && Wire.sent[2]==1 && Wire.sent[3]==0);
    Wire.responder=[](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(r[1],1));
        Wire.responses.push_back(frame(r[1],2));
    };
    assert(c.toggle_status_led_state());
    Wire.responder=[](const std::vector<uint8_t>&) { Wire.responses.push_back(frame(0x25,3)); };
    assert(!c.toggle_status_led_state() && !c.ready());
    assert(c.lastError().failure==KinisiFailure::MALFORMED_RESPONSE);
}
/** Timeouts are bounded across millis rollover and do not resend side-effecting commands. */
static void timeouts_and_bus_errors() {
    reset(); KinisiController c; assert(c.begin(10));
    fake_millis=UINT32_MAX-3; Wire.responder=nullptr;
    size_t before=Wire.transactions;
    assert(!c.stop_motor(0) && !c.ready());
    assert(c.lastError().failure==KinisiFailure::TIMEOUT && Wire.transactions==before+1);
    assert(fake_millis==6);
    assert(!c.stop_motor(0) && Wire.transactions==before+1);
    reset(); KinisiController n; Wire.nack=true;
    assert(!n.begin() && n.lastError().failure==KinisiFailure::TRANSPORT);
    reset(); KinisiController s; Wire.short_write=true;
    assert(!s.begin() && Wire.transactions==0);
    reset(); KinisiController t; Wire.inject_timeout=true;
    assert(!t.begin() && t.lastError().failure==KinisiFailure::TRANSPORT);
}
/** Reject missing READY, incompatible firmware, and a truncated declared frame. */
static void invalid_handshakes() {
    reset(); KinisiController c;
    Wire.responder=[](const std::vector<uint8_t>& r) { identity(r); Wire.responses.pop_back(); };
    assert(!c.begin(5) && !c.ready() && c.lastError().failure==KinisiFailure::TIMEOUT);
    reset(); KinisiController old;
    Wire.responder=[](const std::vector<uint8_t>& r) { identity(r); Wire.responses.front()[8]=1; };
    assert(!old.begin() && old.lastError().failure==KinisiFailure::INCOMPATIBLE_PROTOCOL);
    reset(); KinisiController bad; assert(bad.begin());
    Wire.responder=[](const std::vector<uint8_t>& r) {
        auto f=frame(r[1],2); f[0]=63; Wire.responses.push_back(f);
    };
    assert(!bad.stop_motor(0) && bad.lastError().failure==KinisiFailure::MALFORMED_RESPONSE);
}
/** Check buffer limits before transmitting and full-size structured response decoding. */
static void wire_capacity() {
    reset(); KinisiController c; assert(c.begin());
    Wire.responder=[](const std::vector<uint8_t>& r) {
        std::vector<uint8_t> body(57,0);
        body[0]=2; body[7]=0xf0; body[8]=0x3f; // kp = 1.0
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3])<<8,body));
    };
    size_t before=Wire.transactions;
    auto result=c.get_motor_controller_state(2);
#if KINISI_WIRE_BUFFER_SIZE < 61
    (void)result;
    assert(c.lastError().failure==KinisiFailure::FRAME_TOO_LARGE && c.ready());
    assert(Wire.transactions==before);
    assert(!c.initialize_mecanum_platform(false,false,false,false,false,false,false,false,0.1,0.2,0.1,1000));
    assert(Wire.transactions==before);
#else
    assert(Wire.transactions==before+1 && c.lastError().failure==KinisiFailure::NONE);
    assert(result.motor_index==2 && result.kp==1.0);
#endif
}
/** Golden binary64 encodings include signs, infinities and values spanning integer widths. */
static void floating_point() {
    uint8_t bytes[8];
    kinisi_codec::writeDouble(bytes,-0.0);
    assert(kinisi_codec::readUnsigned(bytes,8)==UINT64_C(0x8000000000000000));
    assert(signbit(kinisi_codec::readDouble(bytes)));
    kinisi_codec::writeDouble(bytes,std::numeric_limits<double>::infinity());
    assert(kinisi_codec::readUnsigned(bytes,8)==UINT64_C(0x7ff0000000000000));
    kinisi_codec::writeUnsigned(bytes,UINT64_C(0x3fb999999999999a),8);
    assert(fabs(kinisi_codec::readDouble(bytes)-0.1)<1e-15);
}
/** Run host tests against the production command and session implementation. */
int main() {
    handshake_and_encoding(); odometry_and_error(); correlation();
    timeouts_and_bus_errors(); invalid_handshakes(); wire_capacity(); floating_point();
    puts("PASS Arduino protocol: INIT/READY, codecs, errors, IDs, deadlines and Wire limits");
}

// File: tests/test_firmware.cpp
// Arduino runtime interoperability with production C firmware protocol/session sources.
#include "kinisi.h"
#include <assert.h>
#include <stdio.h>
extern "C" {
void bridge_reset(void);
void bridge_send(const uint8_t*, size_t);
size_t bridge_read(uint8_t*);
void bridge_advance(uint32_t);
unsigned bridge_stops(void);
unsigned bridge_events(void);
void bridge_set_angle(double);
}
TwoWire Wire;
MockSerial Serial;
uint32_t fake_millis=0;

/** Exchange INIT/READY, ACK, scalar data, ERROR and interval updates across both implementations. */
int main() {
    bridge_reset();
    Wire.responder=[](const std::vector<uint8_t>& r) {
        bridge_send(r.data(), r.size());
        uint8_t data[256];
        size_t n;
        while ((n=bridge_read(data))) Wire.responses.push_back(std::vector<uint8_t>(data,data+n));
    };
    KinisiController c;
    assert(c.begin() && c.ready());
    assert(c.boardInfo().protocol_major==2 && c.boardInfo().board_model==1);
    assert(c.boardProfile() && c.boardProfile()->minor == 3);
    assert(c.boardProfile()->resources.gpio == 10);
    c.get_temperature(0);
    assert(c.lastError().failure == KinisiFailure::SENSOR_NOT_AVAILABLE && c.ready());
    auto clock=c.get_time_status();
    assert(c.lastError().failure==KinisiFailure::NONE);
    assert(clock.clock_mode==0 && clock.clock_quality==1 && clock.interval_ms==30000);
    assert(c.stop_motor(0));
    assert(c.get_encoder_value(0)==0xbeef);
    c.get_platform_odometry();
    assert(c.lastError().code==KinisiErrorCode::ODOMETRY_NOT_INITIALIZED);
    assert(!c.set_time_sync_interval(0) && c.lastError().code==KinisiErrorCode::INVALID_ARGUMENT);
    assert(c.set_time_sync_interval(10000));
    assert(c.get_time_status().interval_ms==10000);
    assert(c.begin());
    assert(c.get_time_status().interval_ms==30000);
    assert(c.subscribe_odometry(0,100));
    bridge_advance(100000); fake_millis += 100;
    assert(c.poll());
    encoder_odometry_sample sample;
    assert(c.getSubscribedEncoderOdometry(0,sample));
    assert(sample.clock_mode == 0 && sample.angle == 1.25 && sample.timestamp_us % 20000 == 0);
    assert(c.unsubscribe_odometry(0));
    assert(!c.getSubscribedEncoderOdometry(0,sample));
    assert(c.subscribe(KinisiTopic::ENCODER_ODOMETRY, 0, 100));
    bridge_advance(100000); fake_millis += 100;
    assert(c.poll());
    KinisiTelemetrySample telemetry;
    assert(c.getSubscriptionSample(KinisiTopic::ENCODER_ODOMETRY, 0, telemetry));
    assert(telemetry.data.encoder_odometry.angle == 1.25);
    assert(telemetry.data.encoder_odometry.clock_quality == 1);
    assert(c.unsubscribe(KinisiTopic::ENCODER_ODOMETRY, 0));
    assert(!c.getSubscriptionSample(KinisiTopic::ENCODER_ODOMETRY, 0, telemetry));
    assert(c.subscribe(KinisiTopic::ENCODER_ODOMETRY, 0, 100, KinisiSubscriptionMode::ON_CHANGE));
    assert(c.ping()); // Drain an initial event queued immediately after the subscribe ACK.
    assert(c.getSubscriptionSample(KinisiTopic::ENCODER_ODOMETRY, 0, telemetry));
    unsigned events = bridge_events();
    bridge_advance(100000); fake_millis += 100; assert(c.poll());
    assert(bridge_events() == events);
    bridge_set_angle(2.5);
    bridge_advance(100000); fake_millis += 100; assert(c.poll());
    assert(bridge_events() == events + 1);
    assert(c.getSubscriptionSample(KinisiTopic::ENCODER_ODOMETRY, 0, telemetry));
    assert(telemetry.data.encoder_odometry.angle == 2.5);
    assert(c.unsubscribe(KinisiTopic::ENCODER_ODOMETRY, 0));
    assert(c.subscribe(KinisiTopic::ENCODER_ODOMETRY, 0, 100, KinisiSubscriptionTrigger::PERIODIC, KinisiSubscriptionDelivery::ON_CHANGE, 200));
    assert(c.ping());
    events = bridge_events();
    bridge_advance(100000); fake_millis += 100; assert(c.poll());
    assert(bridge_events() == events);
    bridge_advance(100000); fake_millis += 100; assert(c.poll());
    assert(bridge_events() == events + 1);
    assert(c.getSubscriptionSample(KinisiTopic::ENCODER_ODOMETRY, 0, telemetry));
    assert(telemetry.data.encoder_odometry.angle == 2.5);
    assert(c.unsubscribe(KinisiTopic::ENCODER_ODOMETRY, 0));
    unsigned previous = bridge_stops();
    bridge_advance(500000); fake_millis += 500;
    assert(!c.poll() && !c.ready());
    assert(bridge_stops() == previous + 1 && c.lastError().code == KinisiErrorCode::INIT_REQUIRED);
    puts("PASS Arduino SDK interoperates with production C firmware INIT, READY, ACK, ERROR and clock status");
}

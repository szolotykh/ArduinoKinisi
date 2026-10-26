#include <kinisi.h>
KinisiController controller;
uint64_t last_timestamp = 0;
bool have_sample = false;

void setup() {
    Serial.begin(9600);
    if (!controller.begin()) return;
    if (controller.supportsTopic(KinisiTopic::TEMPERATURE))
        controller.subscribe(KinisiTopic::TEMPERATURE, 0, 250);
}

void loop() {
    if (!controller.poll()) return;
    KinisiTelemetrySample sample;
    if (controller.getSubscriptionSample(KinisiTopic::TEMPERATURE, 0, sample) &&
            (!have_sample || sample.data.temperature.timestamp_us != last_timestamp)) {
        last_timestamp = sample.data.temperature.timestamp_us;
        have_sample = true;
        Serial.println(sample.data.temperature.celsius);
    }
}

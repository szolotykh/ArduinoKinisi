// Read fitted sensors without enabling motors. Keep poll() running for heartbeats.
#include <kinisi.h>

KinisiController controller(8);

void setup() {
    Serial.begin(115200);
    if (!controller.begin()) {
        Serial.println("Controller initialization failed");
        return;
    }
    const KinisiBoardProfile* profile = controller.boardProfile();
    if (!profile) {
        Serial.println("Unknown board profile; update the library");
        return;
    }
    Serial.print("Board revision: ");
    Serial.print(profile->major); Serial.print('.');
    Serial.print(profile->minor); Serial.print('.'); Serial.println(profile->patch);
    if (profile->hasSensor(KinisiSensor::IMU) && !controller.supportsSensor(KinisiSensor::IMU))
        Serial.println("IMU needs firmware 2.5+ and a real Wire buffer of at least 52 bytes");
    if (profile->hasSensor(KinisiSensor::TEMPERATURE) && !controller.supportsSensor(KinisiSensor::TEMPERATURE))
        Serial.println("Temperature needs firmware 2.4+");
}

void loop() {
    if (!controller.poll()) return;
    static uint32_t last_read = 0;
    if (uint32_t(millis() - last_read) < 1000) return;
    last_read = millis();
    const KinisiBoardProfile* profile = controller.boardProfile();
    if (!profile) return;
    for (uint8_t i = 0; i < profile->temperature_count; ++i) {
        uint8_t id = profile->temperature_ids[i];
        if (!controller.supportsSensor(KinisiSensor::TEMPERATURE, id)) continue;
        double value = controller.get_temperature(id);
        if (controller.lastError().failure == KinisiFailure::NONE) {
            Serial.print("Temperature (C): "); Serial.println(value);
        } else {
            Serial.print("Temperature error: "); Serial.println(uint8_t(controller.lastError().code));
        }
    }
    for (uint8_t i = 0; i < profile->imu_count; ++i) {
        uint8_t id = profile->imu[i].id;
        if (!controller.supportsSensor(KinisiSensor::IMU, id)) continue;
        imu_sample sample = controller.get_imu(id);
        if (controller.lastError().failure == KinisiFailure::NONE) {
            Serial.print("Acceleration X/Y/Z (m/s^2): ");
            Serial.print(sample.acceleration_x); Serial.print(", ");
            Serial.print(sample.acceleration_y); Serial.print(", "); Serial.println(sample.acceleration_z);
            Serial.print("Angular velocity X/Y/Z (rad/s): ");
            Serial.print(sample.angular_velocity_x); Serial.print(", ");
            Serial.print(sample.angular_velocity_y); Serial.print(", "); Serial.println(sample.angular_velocity_z);
        } else {
            Serial.print("IMU error: "); Serial.println(uint8_t(controller.lastError().code));
        }
    }
}

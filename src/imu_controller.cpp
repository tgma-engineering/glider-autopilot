#include "imu_controller.h"

ImuController::ImuController() : bno_(kId, kAddress) {
    sleep_time_ = kSampleDelay;
    new_data_ready_ = false;
}

int8_t ImuController::setup() {
    if (!bno_.begin())
        return -1;
    return 0;
}

int8_t ImuController::loop(uint32_t dt) {
    if (sleep_time_ >= kSampleDelay) {
        // Read data from sensor
        imu::Quaternion attitude = bno_.getQuat();
        attitude_ = Quaterniond(attitude.w(), attitude.x(), attitude.y(), attitude.z());
        imu::Vector<3> acceleration = bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        acceleration_ = Vector3d(acceleration(0), acceleration(1), acceleration(2));
        imu::Vector<3> ang_velocity = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        ang_velocity_ = Vector3d(ang_velocity(0), ang_velocity(1), ang_velocity(2));
        imu::Vector<3> gravity = bno_.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        gravity_ = Vector3d(gravity(0), gravity(1), gravity(2));

        sleep_time_ -= kSampleDelay;
        new_data_ready_ = true;
    }
    sleep_time_ += dt;
    return 0;
}

// Returns if IMU received new data since the last call of this method
bool ImuController::new_data_ready() {
    if (new_data_ready_) {
        new_data_ready_ = false;
        return true;
    } else {
        return false;
    }
}
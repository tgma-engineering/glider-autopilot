#include "imu_controller.h"

ImuController::ImuController() : bno_(kId, kAddress) {
    sleep_time_ = kSampleDelay;
}

int8_t ImuController::setup() {
    if (!bno_.begin())
        return -1;
    return 0;
}

int8_t ImuController::loop(uint32_t dt) {
    if (sleep_time_ >= kSampleDelay) {
        // Read data from sensor
        attitude_ = bno_.getQuat();
        acceleration_ = bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        ang_velocity_ = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        gravity_ = bno_.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

        sleep_time_ -= kSampleDelay;
    }
    sleep_time_ += dt;
    return 0;
}
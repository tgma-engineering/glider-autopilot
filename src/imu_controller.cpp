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
        attitude_ = bno_.getQuat();  // Read data from sensor

        sleep_time_ -= kSampleDelay;
    }
    sleep_time_ += dt;
    return 0;
}
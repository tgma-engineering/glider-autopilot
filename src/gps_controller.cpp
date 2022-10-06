#include "gps_controller.h"

GpsController::GpsController() {
    is_valid_ = false;
}

int8_t GpsController::setup() {
    Serial2.begin(kBaud, SERIAL_8N1, kRxIo, kTxIo);
    return 0;
}

int8_t GpsController::loop(uint32_t dt) {
    if (Serial2.available() > 0) {
        if (gps_.encode(Serial2.read())) {
            if (gps_.satellites.isValid() && gps_.location.isValid() && gps_.altitude.isValid()) {
                satellites_ = gps_.satellites.value();
                latitude_ = gps_.location.lat();
                longitude_ = gps_.location.lng();
                altitude_ = gps_.altitude.meters();
            }
        }
    }

    if (gps_.satellites.age() > kMaxValidTime || gps_.location.age() > kMaxValidTime || gps_.altitude.age() > kMaxValidTime) {
        is_valid_ = false;
    } else {
        is_valid_ = true;
    }

    return 0;
}
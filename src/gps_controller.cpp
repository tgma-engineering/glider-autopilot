#include "gps_controller.h"

GpsController::GpsController() {
    is_valid_ = false;
    is_ref_ = false;
    new_data_ready_ = false;
    needs_flush_ = false;
}

int8_t GpsController::setup() {
    Serial2.begin(kBaud, SERIAL_8N1, kRxIo, kTxIo);  // UART2
    return 0;
}

int8_t GpsController::loop(uint32_t dt) {
    if (needs_flush_) {
        serial2_flush_();
        needs_flush_ = false;
    }

    while (Serial2.available() > 0) {
        if (gps_.encode(Serial2.read())) {
            if (gps_.satellites.isValid() && gps_.location.isValid() && gps_.altitude.isValid() && gps_.satellites.value() >= kMinSatellites) {
                if (gps_.location.isUpdated() && gps_.altitude.isUpdated()) {
                    satellites_ = gps_.satellites.value();
                    latitude_ = gps_.location.lat();
                    longitude_ = gps_.location.lng();
                    altitude_ = gps_.altitude.meters();
                    new_data_ready_ = true;

                    if (!is_ref_ && gps_.location.age() < kMaxValidTime && gps_.altitude.age() < kMaxValidTime && satellites_ >= kMinSatellites) {
                        ref_latitude_ = latitude_;
                        ref_longitude_ = longitude_;
                        ref_altitude_ = altitude_;
                        is_ref_ = true;
                        is_valid_ = true;
                    }

                    // Get time without checking its validity separately
                    year_ = gps_.date.year();
                    month_ = gps_.date.month();
                    day_ = gps_.date.day();
                    time_ = gps_.time.value();
                }
            }
        }
    }

    if (gps_.location.age() > kMaxValidTime || gps_.altitude.age() > kMaxValidTime || satellites_ < kMinSatellites) {
        if (is_valid_) {
            is_valid_ = false;
            Serial.println("Warning: GPS Connection lost");
        }
    } else {
        if (!is_valid_) {
            is_valid_ = true;
            Serial.println("GPS Connection established");
        }
    }

    return 0;
}

void GpsController::flush_serial() {
    needs_flush_ = true;
}

Vector3d GpsController::position() const {
    if (!is_ref_) {
        Serial.println("Warning: There is no valid position information yet");
        return Vector3d::Zero();
    }

    if (!is_valid_) {
        Serial.println("Warning: The position information might not be valid");
    }
    
    Vector3d sph{{latitude_, longitude_, altitude_}};
    return sph_to_cart(sph);
}

void GpsController::time(uint16_t& year, uint8_t& month, uint8_t& day, uint32_t& time) const {
    year = year_;
    month = month_;
    day = day_;
    time = time_;
}

bool GpsController::new_data_ready() {
    if (new_data_ready_) {
        new_data_ready_ = false;
        return true;
    } else {
        return false;
    }
}

Vector3d GpsController::sph_to_cart(const Vector3d& sph) const {
    if (!is_ref_) {
        Serial.println("Warning: No Reference point was set yet");
        return Vector3d(0., 0., 0.);
    }

    double latitude = sph(0);
    double longitude = sph(1);
    if (longitude < ref_longitude_ - 180.) {
        longitude += 360.;
    } else if (longitude > ref_longitude_ + 180.) {
        longitude -= 360.;
    }
    double altitude = sph(2);

    double radius = kSeaLevel + ref_altitude_;
    double m_per_deg = radius * DEG_TO_RAD;

    double z = altitude - ref_altitude_;
    double y = (latitude - ref_latitude_) * m_per_deg;
    double x = (longitude - ref_longitude_) * cos(ref_latitude_) * m_per_deg;
    return Vector3d(x, y, z);
}

Vector3d GpsController::cart_to_sph(const Vector3d& cart) const {
    if (!is_ref_) {
        Serial.println("Warning: No Reference point was set yet");
        return Vector3d(0., 0., 0.);
    }

    double x = cart(0);
    double y = cart(1);
    double z = cart(2);
    
    double radius = kSeaLevel + ref_altitude_;
    double deg_per_m = 1. / radius / DEG_TO_RAD;

    double altitude = z + ref_altitude_;
    double latitude = y * deg_per_m + ref_latitude_;
    double longitude = x * deg_per_m / cos(ref_latitude_) + ref_longitude_;
    if (longitude > 180.) {
        longitude -= 360.;
    } else if (longitude < -180.) {
        longitude += 360.;
    }

    return Vector3d(latitude, longitude, altitude);
}

void GpsController::serial2_flush_() {
    char temp;
    while (Serial2.available() > 0) {
        temp = Serial2.read();
    }
}
#include "gps_controller.h"

const double GpsController::kSeaLevel = 6371001.0;

GpsController::GpsController() {
    is_valid_ = false;
    is_ref_ = false;
}

int8_t GpsController::setup() {
    Serial2.begin(kBaud, SERIAL_8N1, kRxIo, kTxIo);  // UART2
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

                if (!is_ref_ && gps_.satellites.age() < kMaxValidTime && gps_.location.age() < kMaxValidTime && gps_.altitude.age() < kMaxValidTime) {
                    ref_latitude_ = latitude_;
                    ref_longitude_ = longitude_;
                    ref_altitude_ = altitude_;
                    is_ref_ = true;
                }
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

Vector3d GpsController::sph_to_cart(Vector3d sph) {
    if (!is_ref_) {
        Serial.println("Warning: No Reference point was set yet");
        return Vector3d(0., 0., 0.);
    }

    double latitude = sph(1);
    double longitude = sph(2);
    if (longitude < ref_longitude_ - 180.) {
        longitude += 360.;
    } else if (longitude > ref_longitude_ + 180.) {
        longitude -= 360.;
    }
    double altitude = sph(3);

    double radius = kSeaLevel + ref_altitude_;
    double m_per_deg = radius * DEG_TO_RAD;

    double z = altitude - ref_altitude_;
    double y = (latitude - ref_latitude_) * m_per_deg;
    double x = (longitude - ref_longitude_) * cos(ref_latitude_) * m_per_deg;

    return Vector3d(x, y, z);
}

Vector3d GpsController::cart_to_sph(Vector3d cart) {
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
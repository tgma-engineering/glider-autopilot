#ifndef GPS_CONTROLLER_H_
#define GPS_CONTROLLER_H_

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "controller.h"
#include "TinyGPS++.h"

using namespace Eigen;

class GpsController : public Controller {
public:
    static const uint8_t kRxIo = 23;
    static const uint8_t kTxIo = 19;
    static const uint32_t kBaud = 9600;
    static const double kSeaLevel;  // In meters

    // Time after which validity expires with no new measurements
    static const uint32_t kMaxValidTime = 1500;  // In ms

    GpsController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    uint8_t satellites() const { return satellites_; }
    double latitude() const { return latitude_; }
    double longitude() const { return longitude_; }
    double altitude() const { return altitude_; }
    bool is_valid() const { return is_valid_; }

    // Takes Vector (latitude (deg), longitude (deg), altitude (m)) and
    // returns cartesian coordinates (m) (x-east, y-north, z-zenith)
    // relative to reference coordinates
    Vector3d sph_to_cart(Vector3d sph);
    // The inverse of sph_to_cart()
    Vector3d cart_to_sph(Vector3d cart);
    
private:
    TinyGPSPlus gps_;

    uint8_t satellites_;  // Number of active satellites
    double latitude_;     // In degrees
    double longitude_;    // In degrees
    double altitude_;     // In meters

    bool is_valid_;

    // This is where our linearized cartesian reference coordinate system has its origin
    bool is_ref_;
    double ref_latitude_;
    double ref_longitude_;
    double ref_altitude_;
};

#endif  // GPS_CONTROLLER_H_
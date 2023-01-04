#ifndef GPS_CONTROLLER_H_
#define GPS_CONTROLLER_H_

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "controller.h"
#include "TinyGPS++.h"

using namespace Eigen;

class GpsController : public Controller {
public:
    static const uint32_t kUpdateRate = 1;  // In Hz (Datasheet says up to 5)
    static const uint8_t kRxIo = 23;
    static const uint8_t kTxIo = 19;
    static const uint32_t kBaud = 9600;
    static constexpr double kSeaLevel = 6371001.0;  // In meters

    // Time after which validity expires with no new measurements
    static const uint32_t kMaxValidTime = 2500;  // In ms
    static const uint8_t kMinSatellites = 4;  // Should be 8 or so

    GpsController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    void flush_serial();
    uint8_t satellites() const { return satellites_; }
    double latitude() const { return latitude_; }
    double longitude() const { return longitude_; }
    double altitude() const { return altitude_; }
    void time(uint16_t& year, uint8_t& month, uint8_t& day, uint32_t& time) const;
    bool is_valid() const { return is_valid_; }
    Vector3d position() const;
    bool new_data_ready();

    // Takes Vector (latitude (deg), longitude (deg), altitude (m)) and
    // returns cartesian coordinates (m) (x-east, y-north, z-zenith)
    // relative to reference coordinates
    Vector3d sph_to_cart(const Vector3d& sph) const;
    // The inverse of sph_to_cart()
    Vector3d cart_to_sph(const Vector3d& cart) const;
    
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

    // Time (UTC)
    uint16_t year_;
    uint8_t month_;
    uint8_t day_;
    uint32_t time_;  // HHMMSSCC

    bool new_data_ready_;

    bool needs_flush_;

    void serial2_flush();
};

#endif  // GPS_CONTROLLER_H_
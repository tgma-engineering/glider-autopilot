#ifndef GPS_CONTROLLER_H_
#define GPS_CONTROLLER_H_

#include <Arduino.h>
#include "controller.h"
#include "TinyGPS++.h"

// TODO: Read and safe data

class GpsController : public Controller {
public:
    static const uint8_t kRxIo = 10;
    static const uint8_t kTxIo = 5;
    static const uint32_t kBaud = 9600;

    static const uint32_t kMaxValidTime = 1500;

    GpsController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    uint8_t satellites() const { return satellites_; }
    double latitude() const { return latitude_; }
    double longitude() const { return longitude_; }
    double altitude() const { return altitude_; }
    bool is_valid() const { return is_valid_; }
    
private:
    TinyGPSPlus gps_;

    uint8_t satellites_;
    double latitude_;
    double longitude_;
    double altitude_;  // In meters

    bool is_valid_;
};

#endif  // GPS_CONTROLLER_H_
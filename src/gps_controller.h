#ifndef GPS_CONTROLLER_H_
#define GPS_CONTROLLER_H_

#include <Arduino.h>
#include "controller.h"
#include "TinyGPS++.h"

class GpsController : public Controller {
public:
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
};

#endif  // GPS_CONTROLLER_H_
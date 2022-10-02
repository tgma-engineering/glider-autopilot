#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Arduino.h>

class Controller {
public:
    virtual int8_t setup() = 0;
    virtual int8_t loop(uint32_t dt) = 0;
    virtual ~Controller() { }
};

#endif  // CONTROLLER_H_
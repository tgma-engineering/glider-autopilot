#ifndef DEBUG_H_
#define DEBUG_H_

#include <Arduino.h>
#include <ArduinoEigen.h>

using namespace Eigen;

class Debug {
public:
    static void send_float(float f);
    static void send_quaternion(const Quaterniond& q, char start = 's');
};

#endif  // DEBUG_H_
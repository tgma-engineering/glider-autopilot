#include "debug.h"

// Split float into 4 bytes using pointer arithmetic and send them
void Debug::send_float(float f) {
    int8_t* pt = reinterpret_cast<int8_t*>(&f);
    for (int i = 0; i < 4; ++i) {
        Serial.write(*pt++);
    }
}

// For debug purposes, sends quaternion information to matlab script
void Debug::send_quaternion(const Quaterniond& q, char start) {
    Serial.write(start);  // Start Symbol

    float w = static_cast<float>(q.w());
    float x = static_cast<float>(q.x());
    float y = static_cast<float>(q.y());
    float z = static_cast<float>(q.z());

    send_float(w);
    send_float(x);
    send_float(y);
    send_float(z);
}
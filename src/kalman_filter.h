#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Arduino.h>
#include <imumaths.h>

class KalmanFilter {
public:
    KalmanFilter();

    void propagate(double dt);
    void update();
};

#endif  // KALMAN_FILTER_H_
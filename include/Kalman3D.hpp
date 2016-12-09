#ifndef KALMAN3D_H
#define KALMAN3D_H

#include "opencv2/video/tracking.hpp"
//#include "opencv2/highgui/highgui.hpp"

class Kalman3D
{
private:
    cv::KalmanFilter kf_;
    cv::Mat state_, measure_;
    int type_;



public:

    static const u_int8_t STATE_SIZE;
    static const u_int8_t MEASUREMENT_SIZE;

    Kalman3D();
};

#endif // KALMAN3D_H

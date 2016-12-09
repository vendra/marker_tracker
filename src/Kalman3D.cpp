/* Federico Vendramin
 *
 *
 *
 */


#include "Kalman3D.hpp"

const u_int8_t Kalman3D::MEASUREMENT_SIZE = 6;
const u_int8_t Kalman3D::STATE_SIZE = 3;

Kalman3D::Kalman3D()
{
    type_ = CV_32F;
    kf_ = cv::KalmanFilter(Kalman3D::STATE_SIZE, Kalman3D::MEASUREMENT_SIZE, 0, type_);

    state_ = cv::Mat(Kalman3D::STATE_SIZE, 1, type_);
    measure_ = cv::Mat(Kalman3D::MEASUREMENT_SIZE, 1, type_);

    cv::setIdentity(kf_.transitionMatrix);
    kf_.measurementMatrix = cv::Mat::zeros(Kalman3D::MEASUREMENT_SIZE, Kalman3D::STATE_SIZE, type_);
//    kf_.measurementMatrix.at[0,0] = 1.0f;
//    kf_.measurementMatrix.at(7) = 1.0f;
//    kf_.measurementMatrix.at(14) = 1.0f;


}


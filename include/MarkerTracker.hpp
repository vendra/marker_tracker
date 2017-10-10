/** Federico Vendramin 8 Maggio 2016
 *
 *  MarkerTracker.hpp
 *
 *
 *
 */

#ifndef MARKERTRACKER_H_
#define MARKERTRACKER_H_

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>


class MarkerTracker
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    cv::Mat frame_, depth_frame_, im_with_keypoints_;
    cv::Mat cameraMatrix, distCoeffs;
    float X,Y,Z;
    bool camera_info_flag_;
    std::vector<cv::KeyPoint> keypoints_;
    std::string image_path_, depth_path_;
    cv::SimpleBlobDetector::Params params;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    std::vector<cv::Point2f> maskPoints;

    void applyMask(); //private

public:
    MarkerTracker(std::string image_path, std::string depth_path,
                  std::string param_path, std::string calib_path);

    //Loads class parameter from .YAML file
    bool readInputParams(std::string path);//Should be private?

    //Loads intrinsic camera parameters from .YAML 
    bool readCameraParams(std::string path);

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void depthCb(const sensor_msgs::ImageConstPtr& msg); //useless now

    void setMask(const std::vector<cv::Point2f> mask);    

    // Segment the IR image and find (u,v) coordinates of the marker
    cv::Point2f findMarker();

    cv::Point3f findCoord3D(cv::Point2f point);

    bool hasIR();

    bool hasDepth();

    void getIRFrame(cv::Mat& image);

    void getDepthFrame(cv::Mat& depth);

    void setDepthFrame(const cv::Mat& depth);

    void getOutputFrame(cv::Mat& out);

};

#endif // MARKERTRACKER_H

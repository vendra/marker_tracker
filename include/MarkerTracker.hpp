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
    image_transport::Publisher image_pub_;
    ros::Subscriber info_sub_;
    cv::Mat frame_, depth_frame_;
    cv::Mat im_with_keypoints_;
    cv::Mat D,K,R,P;
    float f_x, f_y, c_x, c_y;
    float X,Y,Z;
    bool flag;

    static const std::string IR_WINDOW;
    static const std::string DEPTH_WINDOW;
    static const std::string OUTPUT_WINDOW;

public:
    MarkerTracker(std::string image_path, std::string depth_path);

    ~MarkerTracker();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void depthCb(const sensor_msgs::ImageConstPtr& msg);

    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg);

    // Segment the IR image and find (u,v) coordinates of the marker
    cv::Point2f findMarker();

    cv::Point3f findCoord3D(cv::Point2f point);

    void visualize();

};

#endif // MARKERTRACKER_H

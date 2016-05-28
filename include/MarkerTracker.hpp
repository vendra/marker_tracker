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
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat frame_, depth_frame_;


public:
    MarkerTracker();

    ~MarkerTracker();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void depthCb(const sensor_msgs::ImageConstPtr& msg);

    void compute();

};

#endif // MARKERTRACKER_H

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


class MarkerTracker
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_left_;
    image_transport::Subscriber image_sub_right_;
    image_transport::Publisher image_pub_;


public:
    MarkerTracker();

    ~MarkerTracker();

    void imageCbLeft(const sensor_msgs::ImageConstPtr& msg);

    void imageCbRight(const sensor_msgs::ImageConstPtr& msg);

    void compute();

};

#endif // MARKERTRACKER_H

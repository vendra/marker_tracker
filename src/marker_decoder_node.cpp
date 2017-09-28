#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MarkerTracker.hpp"
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

cv_bridge::CvImagePtr cv_ptr;
image_transport::Publisher depth_pub;

void depthCb(const sensor_msgs::ImageConstPtr&  msg)
{
    try
    {
        //Decode
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		//cv::imshow("view", cv_bridge::toCvShare(msg, "16UC1")->image);
		//cv::waitKey(30);
		//REPUBLISH
		sensor_msgs::ImagePtr msg_to_send = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_ptr->image).toImageMsg();
		depth_pub.publish(msg_to_send);
		
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "marker_decoder_node");
	ros::NodeHandle nh("~");

	//Read ID
    std::string id = argv[1];
    nh.getParam("id", id); 

	//Read CompressedDepth and republish.. as raw
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber depth_sub = it.subscribe("/"+id+"/depth/image", 5, &depthCb);
	depth_pub = it.advertise("/"+id+"/depth/image", 1);

	ros::spin();


	return 0;
}
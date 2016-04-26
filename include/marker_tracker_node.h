/*
 *  Copyright (c) 2015-, NOME <E-MAIL>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MARKER_TRACKER_NODE_H_
#define MARKER_TRACKER_NODE_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>

namespace unipd
{
namespace euroc
{

class ClassTemplate
{
public:

  ClassTemplate (const ros::NodeHandle & node_handle);

  bool
  initialize ();

  void
  imageCallback (const sensor_msgs::Image::ConstPtr & image_msg, // *::ConstPtr is a typedef for boost::shared_ptr<const *>
                 const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg); // only for the classes where it is defined

  //void
  //cloudCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_msg);

  void
  spin ();

private:

  std::string m_log; // to be used within std::cout or ROS_*_STREAM calls
                     // (e.g. std::cout << m_log << "something" << std::endl;)

  ros::NodeHandle m_node_handle;
  image_transport::ImageTransport m_image_transport;

  image_transport::CameraSubscriber m_image_sub; // Subscribe to both image and camera_info topics
  ros::Subscriber m_cloud_sub; // Subscribe to a point cloud topic

  ros::Publisher m_pub; // Publish something

};

} // namespace euroc
} // namespace unipd
#endif // MARKER_TRACKER_NODE_H_

/*  Elaborazione dei dati tridimensionali - Università degli studi di Padova
 *
 *  Copyright (c) 2017- VENDRAMIN FEDERICO <federico.vendramin@gmail.com>
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
 *  DISCLAIMED. IN NO EVENT SHALL Federico Vendramin BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <math.h>

tf::StampedTransform k2transf;
tf::StampedTransform k3transf;
tf::StampedTransform k4transf;
ros::Publisher position3D_pub;
ros::Publisher test_pub;

double getMedian(std::vector<double> coordinate)
{
  std::sort(coordinate.begin(), coordinate.end());

  if(coordinate.size() != 0 )
  {
    if (coordinate.size() % 2 == 0)
      return coordinate[coordinate.size()/2];
    else
      return coordinate[floor(coordinate.size()/2)];
  } else {
    std::cout << "Return -1" << std::endl;
    return -1;
  }
}

double getMedium(std::vector<double> coordinate)
{
  double sum = 0;
  for(int i = 0; i < coordinate.size(); ++i)
    sum = sum + coordinate[i];
  return sum/coordinate.size();
}

tf::Vector3 findMedian3D(std::vector<tf::Vector3> positions)
{
  std::vector<double> xPoints;
  std::vector<double> yPoints;
  std::vector<double> zPoints;

  for(int i = 0; i< positions.size(); ++i)
  {
    //std::cout << i << " X: " << positions[i].getX() << " Y :"
    //          << positions[i].getY() << " Z: "
    //          << positions[i].getZ() << std::endl;
    if(positions[i].getX() != -1)
      xPoints.push_back(positions[i].getX());
    if(positions[i].getY() != -1)
      yPoints.push_back(positions[i].getY());
    if(positions[i].getZ() != -1)
      zPoints.push_back(positions[i].getZ());
  }
  std::cout << "Size X: " << xPoints.size() << " Size Y: " << yPoints.size() << " Size Z: " << zPoints.size() << std::endl;
  return tf::Vector3(getMedium(xPoints), getMedium(yPoints), getMedium(zPoints));
}

void positionCb(const geometry_msgs::PointStampedConstPtr pos1, 
                const geometry_msgs::PointStampedConstPtr pos2,
                const geometry_msgs::PointStampedConstPtr pos3,
                const geometry_msgs::PointStampedConstPtr pos4)
{
  //std::cout << "SEQS:: " << pos1->header.seq << " " << pos2->header.seq <<
  //" " << pos3->header.seq << std::endl;

  tf::Vector3 point2, point3, point4;
  point2.setX(pos2->point.x);
  point2.setY(pos2->point.y);
  point2.setZ(pos2->point.z);

  point3.setX(pos3->point.x);
  point3.setY(pos3->point.y);
  point3.setZ(pos3->point.z);

  point4.setX(pos4->point.x);
  point4.setY(pos4->point.y);
  point4.setZ(pos4->point.z);

  tf::Vector3 point1;
  point1.setX(pos1->point.x);
  point1.setY(pos1->point.y);
  point1.setZ(pos1->point.z);
  tf::Vector3 transfPoint2 = k2transf * point2;
  tf::Vector3 transfPoint3 = k3transf * point3;
  tf::Vector3 transfPoint4 = k4transf * point4;
  std::vector<tf::Vector3> pointsVec;
  pointsVec.push_back(point1);
  pointsVec.push_back(point2);
  pointsVec.push_back(point3);
  pointsVec.push_back(point4);

  std::cout << "1 X: " << point1.getX() << " Y: " << point1.getY() << " Z: " << point1.getZ() << std::endl;

  std::cout << "2 X: " << point2.getX() << " new X: " << transfPoint2.getX()
            << " Y: "  << point2.getX() << " new Y: " << transfPoint2.getY()
            << " Z: "  << point2.getX() << " new Z: " << transfPoint2.getZ() << std::endl;

  std::cout << "3 X: " << point3.getX() << " new X: " << transfPoint3.getX()
            << " Y: "  << point3.getY() << " new Y: " << transfPoint3.getY()
            << " Z: "  << point3.getZ() << " new Z: " << transfPoint3.getZ() << std::endl;

  std::cout << "4 X: " << point4.getX() << " new X: " << transfPoint4.getX()
            << " Y: "  << point4.getY() << " new Y: " << transfPoint4.getY()
            << " Z: "  << point4.getZ() << " new Z: " << transfPoint4.getZ() << std::endl;

  //FILTER and median
  tf::Vector3 pointOut = findMedian3D(pointsVec);
  std::cout << "3D Point X:" << pointOut.getX() << " Y: " <<
               pointOut.getY() << " Z: " << pointOut.getZ() << std::endl;

  geometry_msgs::PointStamped msg;
  msg.point.x = pointOut.getX();
  msg.point.y = pointOut.getY();
  msg.point.z = pointOut.getZ();
  msg.header.frame_id = "kinect_01"; // kinect_01 is master

  position3D_pub.publish(msg);

  geometry_msgs::PointStamped msg_test;
  msg_test.point.x = transfPoint2.getX();
  msg_test.point.y = transfPoint2.getY();
  msg_test.point.z = transfPoint2.getZ();
  msg_test.header.frame_id = "kinect_02"; // kinect_01 is master
  test_pub.publish(msg_test);


  //KALMAN

}

int main (int argc , char* argv[])
{
  ros::init(argc, argv, "Master_node");
  ros::NodeHandle nh("~");

  position3D_pub = nh.advertise<geometry_msgs::PointStamped>("position3D", 10);
  test_pub = nh.advertise<geometry_msgs::PointStamped>("test_kinect_02", 10);


  message_filters::Subscriber<geometry_msgs::PointStamped> k1_sub(nh, "/detector_01/position", 1);
  message_filters::Subscriber<geometry_msgs::PointStamped> k2_sub(nh, "/detector_02/position", 1);
  message_filters::Subscriber<geometry_msgs::PointStamped> k3_sub(nh, "/detector_03/position", 1);
  message_filters::Subscriber<geometry_msgs::PointStamped> k4_sub(nh, "/detector_04/position", 1);

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped,
      geometry_msgs::PointStamped,
      geometry_msgs::PointStamped,
      geometry_msgs::PointStamped> PosSyncPolicy;

  message_filters::Synchronizer<PosSyncPolicy> sync(PosSyncPolicy(20), k1_sub, k2_sub, k3_sub, k4_sub);
  sync.registerCallback(boost::bind(&positionCb, _1, _2, _3, _4));


  //Lookup transform from kx to master k1 for example
  tf::TransformListener listener;

  while(nh.ok()) {
    try {
      listener.lookupTransform("kinect_01", "kinect_02",
                               ros::Time(0), k2transf);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    std::cout << "Transform K2 to K1 OK!" << std::endl;
    break;
  }

  while(nh.ok()) {
    try {
      listener.lookupTransform("kinect_01", "kinect_03",
                               ros::Time(0), k3transf);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    std::cout << "Transform K3 to K1 OK!" << std::endl;
    break;
  }

  while(nh.ok()) {
    try {
      listener.lookupTransform("kinect_01", "kinect_04",
                               ros::Time(0), k3transf);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    std::cout << "Transform K4 to K1 OK!" << std::endl;
    break;
  }

  ros::spin();

  return 0;
}

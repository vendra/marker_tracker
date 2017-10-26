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

#include <opencv2/video/tracking.hpp>

#include <math.h>

tf::StampedTransform k2transf;
tf::StampedTransform k3transf;
tf::StampedTransform k4transf;
ros::Publisher position3D_pub;
ros::Publisher positionKF_pub;


tf::Vector3 point1, point2, point3, point4;
tf::Vector3 transfPoint2, transfPoint3, transfPoint4;
std::vector<tf::Vector3> pointsVecTransf, pointsVec;

//Kalman Filter
cv::KalmanFilter KF(9,3,0);
cv::Mat_<float> measurement(3,1);


double getMedium(const std::vector<double> coordinate)
{
  double sum = 0;
  for(int i = 0; i < coordinate.size(); ++i)
    sum = sum + coordinate[i];
  return sum/coordinate.size();
}

tf::Vector3 findMedium3D(std::vector<tf::Vector3> position, std::vector<tf::Vector3> positionTransf)
{
  std::vector<double> xPoints;
  std::vector<double> yPoints;
  std::vector<double> zPoints;

  //Loop and check if the original positions has -1, if it has then its not valid
  for(int i = 0; i< positionTransf.size(); ++i)
  {
    if(position[i].getX() != -1 && position[i].getY() != -1 && position[i].getZ() != -1)
    {
      xPoints.push_back(positionTransf[i].getX());
      yPoints.push_back(positionTransf[i].getY());
      zPoints.push_back(positionTransf[i].getZ());
    }

  }
  return tf::Vector3(getMedium(xPoints), getMedium(yPoints), getMedium(zPoints));
}

void positionCb(const geometry_msgs::PointStampedConstPtr pos1, 
                const geometry_msgs::PointStampedConstPtr pos2,
                const geometry_msgs::PointStampedConstPtr pos3,
                const geometry_msgs::PointStampedConstPtr pos4)
{
  //std::cout << "SEQS:: " << pos1->header.seq << " " << pos2->header.seq <<
  //" " << pos3->header.seq << std::endl;

  point1.setX(pos1->point.x);
  point1.setY(pos1->point.y);
  point1.setZ(pos1->point.z);

  point2.setX(pos2->point.x);
  point2.setY(pos2->point.y);
  point2.setZ(pos2->point.z);

  point3.setX(pos3->point.x);
  point3.setY(pos3->point.y);
  point3.setZ(pos3->point.z);

  point4.setX(pos4->point.x);
  point4.setY(pos4->point.y);
  point4.setZ(pos4->point.z);

  //Apply transform to "kinect_01" frame as master
  transfPoint2 = k2transf * point2;
  transfPoint3 = k3transf * point3;
  transfPoint4 = k4transf * point4;

  pointsVecTransf.push_back(point1);
  pointsVecTransf.push_back(transfPoint2);
  pointsVecTransf.push_back(transfPoint3);
  pointsVecTransf.push_back(transfPoint4);

  pointsVec.push_back(point1);
  pointsVec.push_back(point2);
  pointsVec.push_back(point3);
  pointsVec.push_back(point4);

  //  std::cout << "1 X: " << point1.getX() << " Y: " << point1.getY() << " Z: " << point1.getZ() << std::endl;

  //  std::cout << "2 X: " << point2.getX() << " new X: " << transfPoint2.getX()
  //            << " Y: "  << point2.getX() << " new Y: " << transfPoint2.getY()
  //            << " Z: "  << point2.getX() << " new Z: " << transfPoint2.getZ() << std::endl;

  //  std::cout << "3 X: " << point3.getX() << " new X: " << transfPoint3.getX()
  //            << " Y: "  << point3.getY() << " new Y: " << transfPoint3.getY()
  //            << " Z: "  << point3.getZ() << " new Z: " << transfPoint3.getZ() << std::endl;

  //  std::cout << "4 X: " << point4.getX() << " new X: " << transfPoint4.getX()
  //            << " Y: "  << point4.getY() << " new Y: " << transfPoint4.getY()
  //            << " Z: "  << point4.getZ() << " new Z: " << transfPoint4.getZ() << std::endl;

  //FILTER and median
  tf::Vector3 pointOut = findMedium3D(pointsVec, pointsVecTransf);
  std::cout << "3D Point X:" << pointOut.getX() << " Y: " <<
               pointOut.getY() << " Z: " << pointOut.getZ() << std::endl;

  geometry_msgs::PointStamped msg;
  msg.point.x = pointOut.getX();
  msg.point.y = pointOut.getY();
  msg.point.z = pointOut.getZ();
  msg.header.frame_id = "kinect_01"; // kinect_01 is master

  position3D_pub.publish(msg);

  //KALMAN

}

void singlePointCb(const geometry_msgs::PointStampedConstPtr pos)
{
  std::string frame = pos->header.frame_id;
  if(frame == "kinect_01")
  {
    point1.setX(pos->point.x);
    point1.setY(pos->point.y);
    point1.setZ(pos->point.z);
    pointsVec[0] = point1;
    pointsVecTransf[0] = point1;
  }
  if(frame == "kinect_02")
  {
    point2.setX(pos->point.x);
    point2.setY(pos->point.y);
    point2.setZ(pos->point.z);
    transfPoint2 = k2transf * point2;
    pointsVec[1] = point2;
    pointsVecTransf[1] = transfPoint2;
  }
  if(frame == "kinect_03")
  {
    point3.setX(pos->point.x);
    point3.setY(pos->point.y);
    point3.setZ(pos->point.z);
    transfPoint3 = k3transf * point3;
    pointsVec[2] = point3;
    pointsVecTransf[2] = transfPoint3;
  }
  if(frame == "kinect_04")
  {
    point4.setX(pos->point.x);
    point4.setY(pos->point.y);
    point4.setZ(pos->point.z);
    transfPoint4 = k4transf * point4;
    pointsVec[3] = point4;
    pointsVecTransf[3] = transfPoint4;
  }


  //Update Medium Point
  tf::Vector3 pointOut = findMedium3D(pointsVec, pointsVecTransf);
  std::cout << "3D Point X:" << pointOut.getX() << " Y: " <<
               pointOut.getY() << " Z: " << pointOut.getZ() << std::endl;

  geometry_msgs::PointStamped msg;
  msg.point.x = pointOut.getX();
  msg.point.y = pointOut.getY();
  msg.point.z = pointOut.getZ();
  msg.header.frame_id = "kinect_01"; // kinect_01 is master

  position3D_pub.publish(msg);

  //KALMAN
  if(pointOut.getX() == pointOut.getX())
  {
  cv::Mat prediction = KF.predict();
  std::cout << "Prediction X: " << prediction.at<float>(0) << " Y: " << prediction.at<float>(1)
            << " Z: " << prediction.at<float>(2) << std::endl;

  measurement.at<float>(0) = pointOut.getX();
  measurement.at<float>(1) = pointOut.getY();
  measurement.at<float>(2) = pointOut.getZ();

  cv::Mat estimated = KF.correct(measurement);

  std::cout << "Estimated X: " << estimated.at<float>(0) << " Y: " << estimated.at<float>(1)
            << " Z: " << estimated.at<float>(2) << std::endl;

  msg.point.x = estimated.at<float>(0);
  msg.point.y = estimated.at<float>(1);
  msg.point.z = estimated.at<float>(2);
  positionKF_pub.publish(msg);
  }

}

int main (int argc , char* argv[])
{
  ros::init(argc, argv, "Master_node");
  ros::NodeHandle nh("~");

  position3D_pub = nh.advertise<geometry_msgs::PointStamped>("position3D", 1);
  positionKF_pub = nh.advertise<geometry_msgs::PointStamped>("positionKF", 1);

  //Kalman Filter Initialization
  //KF(9, 3, 0);

  cv::Mat state(9,3, CV_32F);
  cv::Mat processNoise(9, 3, CV_32F);
  measurement = cv::Mat::zeros(3, 1, CV_32F);
  randn( state, cv::Scalar::all(0), cv::Scalar::all(0.1) );
  KF.transitionMatrix = (cv::Mat_<float>(9,9) << 1,0,0,1,0,0,1,0,0,
                                                 0,1,0,0,1,0,0,1,0,
                                                 0,0,1,0,0,1,0,0,1,
                                                 0,0,0,1,0,0,1,0,0,
                                                 0,0,0,0,1,0,0,1,0,
                                                 0,0,0,0,0,1,0,0,1,
                                                 0,0,0,0,0,0,1,0,1,
                                                 0,0,0,0,0,0,0,1,0,
                                                 0,0,0,0,0,0,0,0,1 );


  measurement.setTo(cv::Scalar(0));

  for(int i = 0; i < KF.statePre.rows; ++i )
  {
    KF.statePre.at<float>(i) = 0;
  }

  cv::setIdentity(KF.measurementMatrix);
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
  randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));


  //Initialize pointsVec and pointsVecTransf
  for(int i=0; i<4; ++i)
  {
    pointsVec.push_back(tf::Vector3(0,0,0));
    pointsVecTransf.push_back(tf::Vector3(0,0,0));
  }


  ros::Subscriber detect_01_sub = nh.subscribe("/detector_01/position", 5, &singlePointCb);
  ros::Subscriber detect_02_sub = nh.subscribe("/detector_02/position", 5, &singlePointCb);
  ros::Subscriber detect_03_sub = nh.subscribe("/detector_03/position", 5, &singlePointCb);
  ros::Subscriber detect_04_sub = nh.subscribe("/detector_04/position", 5, &singlePointCb);

  /*
      //ApproximateTime Filter
      message_filters::Subscriber<geometry_msgs::PointStamped> k1_sub(nh, "/detector_01/position", 1);
      message_filters::Subscriber<geometry_msgs::PointStamped> k2_sub(nh, "/detector_02/position", 1);
      message_filters::Subscriber<geometry_msgs::PointStamped> k3_sub(nh, "/detector_03/position", 1);
      message_filters::Subscriber<geometry_msgs::PointStamped> k4_sub(nh, "/detector_04/position", 1);

      typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped,
          geometry_msgs::PointStamped,
          geometry_msgs::PointStamped,
          geometry_msgs::PointStamped> PosSyncPolicy;

      message_filters::Synchronizer<PosSyncPolicy> sync(PosSyncPolicy(50), k1_sub, k2_sub, k3_sub, k4_sub);
      sync.registerCallback(boost::bind(&positionCb, _1, _2, _3, _4));
      */

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
                               ros::Time(0), k4transf);
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

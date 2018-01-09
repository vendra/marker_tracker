/*  Elaborazione dei dati tridimensionali - Università degli studi di Padova
 *
 *  Copyright (c) 2016- VENDRAMIN FEDERICO <federico.vendramin@gmail.com>
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

#include <marker_tracker_node.hpp>
#include <MarkerTracker.hpp>
#include <ros/console.h>
#include <tf/transform_listener.h>


//Enables the setup phase, should be active
#define SETUP

cv::Mat depth_frame;
cv_bridge::CvImagePtr cv_ptr;

void mouseClick(int event, int x, int y, int flags, void* maskPoints)
{
  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    std::cout << "Set mask in position (" << x << ", " << y << ")" << std::endl;
    std::vector<cv::Point2f> *maskPointPtr = static_cast<std::vector<cv::Point2f> *>(maskPoints);
    cv::Point2f pt = cv::Point2f(x,y);
    (*maskPointPtr).push_back(pt);
  }
  else if  ( event == cv::EVENT_RBUTTONDOWN )
  {
    std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
  else if  ( event == cv::EVENT_MBUTTONDOWN )
  {
    std::cout << "Reset mask" << std::endl;
    std::vector<cv::Point2f> *maskPointPtr = static_cast<std::vector<cv::Point2f> *>(maskPoints);
    (*maskPointPtr).resize(0);
  }
  else if ( event == cv::EVENT_MOUSEMOVE )
  {
    //std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;

  }
}

int main (int argc , char* argv[])
{
  ros::init(argc, argv, "Marker_tracker_node");
  ros::NodeHandle nh("~");

  //Read ID
  std::string id = argv[1];

  //Checks for arguments
  bool enableView = true;
  if(argc == 3)
  {
    std::string view = argv[2];
    if(view == "--noview")
      enableView = false; //disable window visualizations
  }

  ros::Publisher position_pub = nh.advertise<geometry_msgs::PointStamped>("position", 10);
  geometry_msgs::PointStamped msg;
  msg.point.x = 0;
  msg.point.y = 0;
  msg.point.z = 0;

  //Loads the correct .YAML file containing correct detector parameters and camera calibration params
  std::string param_path = ros::package::getPath("marker_tracker")+"/parameters.yaml";
  std::string calib_path = ros::package::getPath("marker_tracker")+"/param/"+id+".yaml";
  std::cout << "param path: " << param_path << std::endl;
  std::cout << "calib path: " << calib_path << std::endl;

  MarkerTracker tracker("/"+id+"/ir/image", "/"+id+"/depth/image",
                        param_path, calib_path);

  // wait for images to be published
  ros::spinOnce();
  ros::Duration(2.0).sleep();
  ros::spinOnce();

  cv::Mat frame, out;
  std::vector<cv::Point2f> maskPoints;

  //GUI
  int con_slider = 0;
  int bri_slider = 0;
  cv::startWindowThread();
  cv::namedWindow(id+"Setup");
  cv::setMouseCallback(id+"Setup", mouseClick, &maskPoints);
  cv::createTrackbar("Contrast", id+"Setup", &con_slider, 240);
  cv::createTrackbar("Brightness", id+"Setup", &bri_slider, 100);
  cv::waitKey(30);

  ROS_INFO("Press q to confirm and proceed");

  char c;
  bool exit_key_pressed = false;
#ifdef SETUP

  //Main computation loop
  while (!exit_key_pressed)
  {
    ros::spinOnce();

    //Apply user mask, detect marker and get an image to show the user
    tracker.setMask(maskPoints);
    tracker.findMarker();
    tracker.getOutputFrame(out);

    position_pub.publish(msg);

    //Brightness setup visulization
    if (!out.empty())
    {
      for( int y = 0; y < out.rows; y++)
        for( int x = 0; x < out.cols; x++)
          for(int c = 0; c < 3; c++)
            out.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>((40+con_slider)/(40.0)*( out.at<cv::Vec3b>(y,x)[c]) + bri_slider);
    }

    //Show black mask
    for(int i=0; i < maskPoints.size(); ++i)
      cv::circle(out, maskPoints[i], 3, cv::Scalar(0,0,0), -1);

    //Show setup frame
    cv::imshow(id+"Setup", out);
    c = cv::waitKey(30);

    if (c == 'q')
      exit_key_pressed = true;
    if (c== '.') //To reload detection parameters from.yaml
      tracker.readInputParams(param_path);
  }

  tracker.setMask(maskPoints);
#endif
  cv::destroyWindow(id+"Setup");
  cv::waitKey(30);
  ROS_INFO("Setup Completed");

  // Create new Window for output
  if(enableView)
    cv::namedWindow(id+"Output");
  //cv::createTrackbar("Contrast", id+"Setup", &con_slider, 240);
  cv::waitKey(30);

  cv::KeyPoint imagePoint;
  cv::Point3f spacePoint;
  cv::Mat depth;

  while(nh.ok())
  {
    if(tracker.hasIR()) // Checks frames are being published
    {
      imagePoint = tracker.detectMarker(); // detection
      spacePoint = tracker.findCoord3D(imagePoint); // projection

      if(enableView)
        tracker.getOutputFrame(out);

      //Apply brightness-contrast
      if (enableView && !out.empty())
      {
        for( int y = 0; y < out.rows; y++)
          for( int x = 0; x < out.cols; x++)
            for(int c = 0; c < 3; c++)
              out.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>((40+con_slider)/(40.0)*( out.at<cv::Vec3b>(y,x)[c]) + bri_slider);
      }

      // Publish estimated 3D point
      msg.point.x = spacePoint.x;
      msg.point.y = spacePoint.y;
      msg.point.z = spacePoint.z;
      msg.header.frame_id = id;
      position_pub.publish(msg);

    } // If new frame arrived

    ros::spinOnce();
    if(enableView)
    {
      //imshow("test", depth);
      imshow(id+"Output",out);

      c = cv::waitKey(30);
      if (c == 'q')
        break;
    }
  }

  return 0;
}

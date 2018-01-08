/** Elaborazione dei dati tridimensionali - Università degli studi di Padova
 *
 *  Created 8th May 2016
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

#include <MarkerTracker.hpp>


MarkerTracker::MarkerTracker(std::string image_path, std::string depth_path,
                             std::string param_path, std::string calib_path)
  : it_(nh_)
{
  // Subscribe to input video feed
  image_sub_ = it_.subscribe(image_path, 5, &MarkerTracker::imageCb, this);
  depth_sub_ = it_.subscribe(depth_path, 5, &MarkerTracker::depthCb, this);

  //3D point
  X = 0.0;
  Y = 0.0;
  Z = 0.0;

  newFrame = false;

  image_path_ = image_path;
  depth_path_ = depth_path;

  //Reads calibration from file
  readCameraParams(calib_path);

  //Reads param from file
  readInputParams(param_path);

  //Init Blob Detector
  detector = cv::SimpleBlobDetector::create(params);
}

bool MarkerTracker::readInputParams(std::string path)
{

  std::cout << "Reading blob parameters from input file.. \n";

  cv::FileStorage fs(path, cv::FileStorage::READ);
  if ( !fs.isOpened() )
  {
    std::cout << "Cannot open " << path << std::endl;
    return false;
  }

  fs["minDistBetweenBlobs"] >> params.minDistBetweenBlobs;
  fs["minThreshold"]        >> params.minThreshold;
  fs["maxThreshold"]        >> params.maxThreshold;
  fs["thresholdStep"]       >> params.thresholdStep;
  fs["filterByInertia"]     >> params.filterByInertia;
  fs["minInertiaRatio"]     >> params.minInertiaRatio;
  fs["maxInertiaRatio"]     >> params.maxInertiaRatio;
  fs["filterByConvexity"]   >> params.filterByConvexity;
  fs["minConvexity"]        >> params.minConvexity;
  fs["maxConvexity"]        >> params.maxConvexity;
  fs["filterByColor"]       >> params.filterByColor;
  fs["blobColor"]           >> params.blobColor;
  fs["filterByCircularity"] >> params.filterByCircularity;
  fs["minCircularity"]      >> params.minCircularity;
  fs["maxCircularity"]      >> params.maxCircularity;
  fs["filterByArea"]        >> params.filterByArea;
  fs["minArea"]             >> params.minArea;
  fs["maxArea"]             >> params.maxArea;

  return true;
}

bool MarkerTracker::readCameraParams(std::string path) //make private
{
  std::cout << "Reading camera parameters from input file.. \n";

  cv::FileStorage fs(path, cv::FileStorage::READ);
  if ( !fs.isOpened() )
  {
    std::cout << "Cannot open " << path << std::endl;
    return false;
  }

  fs["cameraMatrix"] >> cameraMatrix;
  fs["distortionCoefficients"]   >> distCoeffs;

  std::cout << "Camera calibration parameters OK!" << std::endl;
  std::cout << "camera matrix: " << cameraMatrix << std::endl;
  std::cout << "distortion coeffs: " << distCoeffs << std::endl;

}

bool MarkerTracker::newFrameArrived(){ return newFrame; }

void MarkerTracker::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  frame_ = cv_ptr->image;
  newFrame = true;

}

void MarkerTracker::depthCb(const sensor_msgs::ImageConstPtr&  msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depth_frame_ = cv_ptr->image;

}

void MarkerTracker::setMask(const std::vector<cv::Point2f> points)
{
  maskPoints = points;
}

//Apply user masking
void MarkerTracker::applyMask()
{   
  //Mask clicked points
  for(int i=0; i<maskPoints.size(); ++i)
    cv::circle(frame_, maskPoints[i], 3, cv::Scalar(0,0,0), -1);
}

//Legacy - Spheric Marker
cv::Point2f MarkerTracker::findMarker()
{
  applyMask();

  cv::Mat img_source = frame_;
  img_source.convertTo(img_source, CV_8UC1, 1.0/256);

  detector->detect(img_source, keypoints_);

  cv::Point2f p(-1.0, -1.0);

  if(keypoints_.size() > 0)
    p = keypoints_[keypoints_.size()-1].pt;

  return p;
}

//New - Cubic Marker
cv::KeyPoint MarkerTracker::detectMarker()
{
  applyMask();
  newFrame = false;
  cv::Mat img_source = frame_;
  img_source.convertTo(img_source, CV_8UC1, 1.0/256);

  findMarkerContours(img_source, contours);

  blobImage = img_source.clone();
  cv::cvtColor(blobImage, blobImage, cv::COLOR_GRAY2BGR);
  //cv::GaussianBlur(blobImage, blobImage, cv::Size(3,3),0, 0);

  //Draw filled contours to improve blob detection
  for( int i = 0; i < contours.size(); i++ )
    cv::drawContours(blobImage, contours, i, cv::Scalar(255,255,255), CV_FILLED);

  //----------------BLOB DETECTOR-------->

  //Detection BLOB
  detector->detect(blobImage, keypoints);
  //cv::imshow("Blob", blobImage);
  //cv::waitKey(30);

  if(keypoints.size() > 0)
    return keypoints[0];
  else
    return cv::KeyPoint(-1.0, -1.0, -1.0); // size is -1 means no detection, check if its an allowed value
}

//private
void MarkerTracker::findMarkerContours( const cv::Mat& image, std::vector<std::vector<cv::Point>>& contours )
{
  contours.clear();
  cv::threshold(image, image, 150, 255, cv::THRESH_BINARY);
  cv::GaussianBlur(image, image, cv::Size(3,3), 0, 0);
  cv::findContours(image, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

}


// Exploit pinhole camera model to compute X and Y, find Z in depth map
cv::Point3f MarkerTracker::findCoord3D(cv::Point2f point)
{
  depthValues.resize(0);

  //Check bounds and select neighbors +-1 in X and Y axis to compute median
  if (point.x > 0 && point.x < frame_.cols && point.y > 0 && point.y < frame_.rows)
  {
    depthValues.push_back(static_cast<float>(depth_frame_.at<unsigned short>(point.x, point.y))/1000);
    depthValues.push_back(static_cast<float>(depth_frame_.at<unsigned short>(point.x+1, point.y))/1000);
    depthValues.push_back(static_cast<float>(depth_frame_.at<unsigned short>(point.x, point.y+1))/1000);
    depthValues.push_back(static_cast<float>(depth_frame_.at<unsigned short>(point.x-1, point.y))/1000);
    depthValues.push_back(static_cast<float>(depth_frame_.at<unsigned short>(point.x, point.y-1))/1000);
  } else {
    return cv::Point3f(0, 0, 0);
  }
  /*
  std::cout << "DepthValues: ";
  for(int i=0; i<depthValues.size(); i++) {
    std::cout << " " << depthValues[i] << " ";
  }
  std::cout << std::endl;
  */

  std::sort (depthValues.begin(), depthValues.end());
  Z = depthValues[2]; //Median, new way

  //unsigned short uZ = depth_frame_.at<unsigned short>(point.x,point.y); // old way
  //Z = static_cast<float>(uZ)/1000; //Old way to get Z

  // compute X e Y by backprojection
  X = (point.x - cameraMatrix.at<double>(0,2)) * Z / cameraMatrix.at<double>(0,0);
  Y = (point.y - cameraMatrix.at<double>(1,2)) * Z / cameraMatrix.at<double>(1,1);

  return cv::Point3f(X,Y,Z);
}

cv::Point3f MarkerTracker::findCoord3D(cv::KeyPoint point)
{
  depthValues.resize(0);

  findMarkerDepth(point);
  Z = medianDepth;
  // compute X,Y by backprojection
  X = (point.pt.x - cameraMatrix.at<double>(0,2)) * Z / cameraMatrix.at<double>(0,0);
  Y = (point.pt.y - cameraMatrix.at<double>(1,2)) * Z / cameraMatrix.at<double>(1,1);

  return cv::Point3f(X,Y,Z);
}

//Computes the Depth of the detected marker. Filter out points lying on the marker's edges.
void MarkerTracker::findDepthValues()
{
  depthValues.resize(0); //resets precedent residual values
  depthPoints.resize(0);

  for(int i = centerX - radius; i < centerX + radius; ++i) {
    for(int j = centerY - radius; j < centerY + radius; ++j) {
      //Scan all points in rectangle, selects points not on a edge
      if (frame_.at<uchar>(j,i) >= 245) {
        continue;
      } else if (pow(centerX - i, 2) + pow(centerY - j, 2) <= pow(radius/2,1.5)) {
        double depthTemp = static_cast<double>(depth_frame_.at<unsigned short>(j,i)) / 1000;
        if (depthTemp != 0) {
          depthValues.push_back(depthTemp);
          depthPoints.push_back(cv::Point(i,j));
        }
      }
    } // nested for
  } // for
}

//private
//Computes the median point
void MarkerTracker::findMedianDepth()
{
  std::sort(depthValues.begin(), depthValues.end());

  medianDepth = -1;

  if(depthValues.size() != 0 )
  {
    if (depthValues.size() % 2 == 0)
      medianDepth = depthValues[depthValues.size()/2];
    else
      medianDepth = depthValues[floor(depthValues.size()/2)];

    //std::cout << "minValue: " << depthValues[0] << " avgValue: " << medianDepth << " maxValue: " << depthValues[depthValues.size()-1] << std::endl;
  }
}

//private
//Refines the median point by removing points too far from the median and computing again
//This can be avoided since the medianDepth is already good enough.
void MarkerTracker::refineMedianDepth()
{
  if(depthValues.size() != 0 )
  {
    if((depthValues[0] < medianDepth - 0.5) || (depthValues[depthValues.size()-1] > medianDepth + 0.5)) {
      refinedValues.resize(0);
      for(int i = 0; i < depthValues.size(); ++i){
        if((depthValues[i] < medianDepth - 0.5) || (depthValues[i] > medianDepth + 0.5))
          continue; // skip the point if lower outlier
        refinedValues.push_back(depthValues[i]);
      }

      std::sort(refinedValues.begin(), refinedValues.end());

      if (refinedValues.size() % 2 == 0)
        medianDepth = refinedValues[refinedValues.size()/2];
      else
        medianDepth = refinedValues[floor(refinedValues.size()/2)];

      //std::cout << "REFINED minValue: " << refinedValues[0] << " avgValue: " << medianDepth << " maxValue: " << refinedValues[refinedValues.size()-1] << std::endl;
    }
  }
}

//Computes the depth of the marker, descripted by a cv::Keypoint
//Assumes it is a valid keypoint of a valid detection
void MarkerTracker::findMarkerDepth(const cv::KeyPoint markerKeypoint)
{
  medianDepth = -1.0;
  radius = markerKeypoint.size;
  centerX = markerKeypoint.pt.x;
  centerY = markerKeypoint.pt.y;
  findDepthValues();
  findMedianDepth();
  refineMedianDepth();
}

bool MarkerTracker::hasIR()
{
  return (!frame_.empty());
}

bool MarkerTracker::hasDepth()
{
  return (!depth_frame_.empty());
}

void MarkerTracker::getIRFrame(cv::Mat &image)
{
  if (frame_.empty())
    ROS_INFO("Empty IR Frame");
  else
    image = frame_;
}

void MarkerTracker::getDepthFrame(cv::Mat &depth)
{
  if (depth_frame_.empty())
    ROS_INFO("Empty Depth Frame!");
  else {
    depth = depth_frame_;
    cv::cvtColor(depth, depth, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < keypoints_.size(); ++i)
      cv::circle(depth, keypoints_[i].pt, 5, cv::Scalar(0,255,0), 2);
  }
}

void MarkerTracker::getOutputFrame(cv::Mat &out)
{
  if (frame_.empty())
  {
    ROS_INFO("Cannot render output image with keypoints");
  }
  else
  {
    cv::Mat im;
    frame_.convertTo(im, CV_8UC1, 1.0/256);
    cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);

    for(int i = 0; i < depthPoints.size(); ++i)
      cv::circle(im, depthPoints[i], 1, cv::Scalar(0, 255, 0));

    cv::drawKeypoints(im , keypoints, im, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    out = im;
  }
}



/** Federico Vendramin 8 Maggio 2016
 *
 *  MarkerTracker.cpp
 *
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

//Should I add a time synchronizer to get the IR and depth synchronized toghether??

void MarkerTracker::setMask(const std::vector<cv::Point2f> points)
{
  maskPoints = points;
}

void MarkerTracker::applyMask()
{   
  //Mask clicked points
  for(int i=0; i<maskPoints.size(); ++i)
    cv::circle(frame_, maskPoints[i], 3, cv::Scalar(0,0,0), -1);
}


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

cv::KeyPoint MarkerTracker::detectMarker()
{
  applyMask();

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

  //BlobDetector parameters should be already be parsed and loaded

  //Detection BLOB
  detector->detect(blobImage, keypoints);

  //cv::KeyPoint p(-1.0, -1.0); // If no detection -1,-1 or something better like NaN?

  if(keypoints.size() > 0)
    return keypoints[0];
  else
    return cv::KeyPoint(-1.0, -1.0, -1.0); // size is -1 means no detection, check if its an allowed value
}

//private
static void MarkerTracker::findMarkerContours( const cv::Mat& image, std::vector<std::vector<cv::Point>>& contours )
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
  std::cout << "DepthValues: ";
  for(int i=0; i<depthValues.size(); i++) {
    std::cout << " " << depthValues[i] << " ";
  }
  std::cout << std::endl;

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

  findMarkerDepth();

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
  std::cout << "DepthValues: ";
  for(int i=0; i<depthValues.size(); i++) {
    std::cout << " " << depthValues[i] << " ";
  }
  std::cout << std::endl;

  std::sort (depthValues.begin(), depthValues.end());
  Z = depthValues[2]; //Median, new way

  //unsigned short uZ = depth_frame_.at<unsigned short>(point.x,point.y); // old way
  //Z = static_cast<float>(uZ)/1000; //Old way to get Z

  // compute X e Y by backprojection
  X = (point.x - cameraMatrix.at<double>(0,2)) * Z / cameraMatrix.at<double>(0,0);
  Y = (point.y - cameraMatrix.at<double>(1,2)) * Z / cameraMatrix.at<double>(1,1);

  return cv::Point3f(X,Y,Z);
}


void MarkerTracker::findDepthValues()
{
  depthValues.resize(0); //resets precedent residual values

  for(int i = centerX - radius; i < centerX + radius; ++i) {
    for(int j = centerY - radius; j < centerY + radius; ++j) {
      //Scan all points in rectangle
      if (frame_.at<uchar>(j,i) >= 245) {
        continue;
      } else if (pow(centerX - i, 2) + pow(centerY - j, 2) <= pow(radius/2,1.5)) {
        double depthTemp = static_cast<double>(depth_.at<unsigned short>(j,i)) / 1000;
        if (depthTemp != 0) {
          depthValues.push_back(depthTemp);
          cv::circle(image, cv::Point(i,j), 1, cv::Scalar(0, 255, 0));
        }
      }
    } // nested for
  } // for
}

//private
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

    std::cout << "minValue: " << depthValues[0] << " avgValue: " << medianDepth << " maxValue: " << depthValues[depthValues.size()-1] << std::endl;
  }
}

//private
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

      std::cout << "REFINED minValue: " << refinedValues[0] << " avgValue: " << medianDepth << " maxValue: " << refinedValues[refinedValues.size()-1] << std::endl;

    }
  }
}


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
    //im = frame_;
    frame_.convertTo(im, CV_8UC1, 1.0/256);

    //cv::drawKeypoints( frame_, keypoints_, im_with_keypoints_, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT );

    // Brighten image to visualize it easily

    /*for( int y = 0; y < im.rows; y++ )
            for( int x = 0; x < im.cols; x++ )
                for( int c = 0; c < 3; c++ )
                    im.at<uchar>(y,x) = cv::saturate_cast<uchar>( 2.2*( im.at<uchar>(y,x)) );
         */
    cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < keypoints_.size(); ++i)
      cv::circle(im, keypoints_[i].pt, 5, cv::Scalar(0,255,0), 2);

    out = im;
  }
}



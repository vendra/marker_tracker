/** Federico Vendramin 8 Maggio 2016
 *
 *  MarkerTracker.cpp
 *
 *
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

// Exploit pinhole camera model to compute X and Y, find Z in depth map
cv::Point3f MarkerTracker::findCoord3D(cv::Point2f point)
{

    // in (u,v) trovo valore di Z corrispondente
    unsigned short uZ = depth_frame_.at<unsigned short>(point.x,point.y);
    Z = static_cast<float>(uZ)/1000;
    
    // compute X e Y
    X = (point.x - cameraMatrix.at<double>(0,2)) * Z / cameraMatrix.at<double>(0,0);
    Y = (point.y - cameraMatrix.at<double>(1,2)) * Z / cameraMatrix.at<double>(1,1);

    return cv::Point3f(X,Y,Z);

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
    else
        depth = depth_frame_;
}

void MarkerTracker::setDepthFrame(const cv::Mat &depth) //remove
{
    depth_frame_ = depth;
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



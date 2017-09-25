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
    // Counting object with static variable, such that every camera has its image displayed in correct window

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(image_path, 5, &MarkerTracker::imageCb, this);
    depth_sub_ = it_.subscribe(depth_path, 5, &MarkerTracker::depthCb, this);
    //info_sub_ = nh_.subscribe("/kinect2_head/depth/camera_info", 5, &MarkerTracker::cameraInfoCb, this);
    //ros::Subscriber depth_sub_tmp = nh_.subscribe(depth_path, 5, &MarkerTracker::depthCb, this);
    //ros::Subscriber ir_sub_tmp = nh_.subscribe(image_path, 5, &MarkerTracker::imageCb, this);

    //camera_info_flag_ = false;

    // Initializes intrinsic params 
    f_x = 0.0; // not used anymore? Use cv::Mat cameraMatrix instead
    f_y = 0.0;
    c_x = 0.0;
    c_y = 0.0;

    //3D point
    X = 0.0;
    Y = 0.0;
    Z = 0.0;

    // no ROI
    roiX_ = 0;
    roiY_ = 0;

    image_path_ = image_path;
    depth_path_ = depth_path;

    //Reads calibration from file
    readCameraParams(calib_path);

    //Reads param from file
    readInputParams(param_path);

    //Init Blob Detector
    detector = cv::SimpleBlobDetector::create(params);
}

MarkerTracker::~MarkerTracker()
{
    //cv::destroyAllWindows();
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

    std::cout << "maxArea: " << params.maxArea << std::endl;

    return true;
}

bool MarkerTracker::readCameraParams(std::string path)
{
    std::cout << "Reading camera parameters from input file.. \n";

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if ( !fs.isOpened() )
    {
        std::cout << "Cannot open " << path << std::endl;
        return false;
    }

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"]   >> distCoeffs;

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

/*void MarkerTracker::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
    if (!camera_info_flag_)
    {
        f_x = msg->K[0];
        f_y = msg->K[4];
        c_x = msg->K[2];
        c_y = msg->K[5];

        std::cout << "Camera calibration parameters OK!" << std::endl;
        std::cout << "f_x: " << f_x << std::endl;
        std::cout << "f_y: " << f_y << std::endl;
        std::cout << "c_x: " << c_x << std::endl;
        std::cout << "c_y: " << c_y << std::endl;
        camera_info_flag_ = true;
    }

}*/


void MarkerTracker::setROI(int x, int y)
{
    roiX_ = x;
    roiY_ = y;
    this->applyROI();

}

void MarkerTracker::applyROI()
{
    for (int j = 0; j < roiX_; j++)
        for (int i = 0; i < roiY_; i++)
            frame_.at<uchar>(i,j) = 0;
}


cv::Point2f MarkerTracker::findMarker()
{
    this->applyROI();
    cv::Mat img_black;
    cv::Mat img_source = frame_;
    img_source.convertTo(img_source, CV_8UC1, 1.0/256);
    
    //cv::Mat img_prova = cv::imread("/home/federico/blob_detection.jpg");

//    cv::SimpleBlobDetector::Params params;
//    params.minDistBetweenBlobs = 20.0f;
//    params.minThreshold = 10; // 100
//    params.maxThreshold = 255; //255

//    params.filterByInertia = false;
//    params.filterByConvexity = false;
//    params.filterByColor = false;
//    params.blobColor = 255; //255
//    params.filterByCircularity = false;

//    params.filterByArea = true;
//    params.minArea = .5; //1
//    params.maxArea = 2.0; // 30

    //cv::Ptr<cv::SimpleBlobDetector> detector_new = cv::SimpleBlobDetector::create(params); //OpencV 3
    //cv::threshold(img_source, img_source, 150, 255, cv::THRESH_BINARY);

    detector->detect(img_source, keypoints_);
    //detector_new->detect(img_source, keypoints_);

    //std::vector<cv::Point2f> points;
    //cv::KeyPoint::convert(points, keypoints_);
    //std::cout << "Mat 8U: " << img_source << std::endl;


    //Debug
    /*
    for (int i = 0; i < img_source.rows; ++i)
        for (int j = 0; j < img_source.cols; ++j)
        {
            if (img_source.at<uchar>(i,j) > 250)
                std::cout << "Trovato pixel alto: " << std::endl;
        }
    */

    //if (keypoints_.size() != 0)
    //    std::cout << "Number of keypoints_: " << keypoints_.size() << std::endl;

    cv::Point2f p;
    for(int i = 0; i < keypoints_.size(); ++i)
        p = keypoints_[i].pt;


    // Ritorna l ultimo keypoint, solitamente però è uno.
    // Aggiungere dei controlli qui
    // Sto ritornando effettivamente il centro del blob? (SI)
    return p;

}

// Exploit pinhole camera model to compute X and Y, find Z in depth map
cv::Point3f MarkerTracker::findCoord3D(cv::Point2f point)
{

    std::cout << "pre uZ\n";
    // in (u,v) trovo valore di Z corrispondente
    unsigned short uZ = depth_frame_.at<unsigned short>(point.x,point.y);
    Z = static_cast<float>(uZ)/1000;

    //if (Z != 0)
    // std::cout << "Z: " << Z << std::endl;

    std::cout << "preCompute XY\n";
    // compute X e Y
    X = (point.x - c_x) * Z / f_x;
    Y = (point.y - c_y) * Z / f_y;
    std::cout << "XY computer\n";

    // ritorno point3f (XYZ)
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
        
         for( int y = 0; y < im.rows; y++ )
            for( int x = 0; x < im.cols; x++ )
                for( int c = 0; c < 3; c++ )
                    im.at<uchar>(y,x) = cv::saturate_cast<uchar>( 2.2*( im.at<uchar>(y,x)) );

        cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < keypoints_.size(); ++i)
          cv::circle(im, keypoints_[i].pt, 5, cv::Scalar(0,255,0), 2);

        out = im;
    }
}



/** Federico Vendramin 8 Maggio 2016
 *
 *  MarkerTracker.cpp
 *
 *
 *
 */

#include <MarkerTracker.hpp>

const std::string MarkerTracker::IR_WINDOW = "IR Window";
const std::string MarkerTracker::DEPTH_WINDOW = "Depth Window";
const std::string MarkerTracker::OUTPUT_WINDOW = "Output Window";

MarkerTracker::MarkerTracker()
    : it_(nh_)
{
    // Subscribe to input video feed and publish output video feed


    image_pub_ = it_.advertise("/image_converter/output_video", 1);



    cv::namedWindow(IR_WINDOW);
    cv::namedWindow(DEPTH_WINDOW);
    cv::namedWindow(OUTPUT_WINDOW);
}

MarkerTracker::~MarkerTracker()
{
    cv::destroyAllWindows();
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

void MarkerTracker::depthCb(const sensor_msgs::ImageConstPtr& msg)
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


cv::Point2f MarkerTracker::findMarker()
{

    int threshold = 80;


    cv::Mat img_black;
    cv::Mat img_source = frame_;
    img_source.convertTo(img_source, CV_8U, 1.0/256);

    cv::Mat img_prova;
    img_prova = cv::imread("/home/federico/blob_detection.jpg");
    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 20.0f;
    params.minThreshold = 200;
    params.maxThreshold = 255;

    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.blobColor = 255;
    params.filterByCircularity = false;

    params.filterByArea = true;
    params.minArea = 5.0;
    params.maxArea = 100.0;

    cv::SimpleBlobDetector detector(params);

    std::vector<cv::KeyPoint> keypoints;

    //cv::threshold(img_source, img_source, 150, 255, cv::THRESH_BINARY);

    detector.detect(img_source, keypoints);


    cv::threshold(img_source, img_black, 255, 255, cv::THRESH_BINARY);


    cv::drawKeypoints( img_black, keypoints, im_with_keypoints_, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    std::vector<cv::Point2f> punti;
    //cv::KeyPoint::convert(punti, keypoints);
    //std::cout << "Mat 8U: " << img_source << std::endl;

    /*
    for (int i = 0; i < img_source.rows; ++i)
        for (int j = 0; j < img_source.cols; ++j)
        {
            if (img_source.at<uchar>(i,j) > 250)
                std::cout << "Trovato pixel alto: " << std::endl;
        }
*/
    //std::cout << "Numero di keypoints: " << keypoints.size() << std::endl;

    cv::Point2f p;
    for(int i = 0; i < keypoints.size(); ++i)
    {
        //std::cout << "disegnato keypoint!" << std::endl;
        p = keypoints[i].pt;
        cv::circle(im_with_keypoints_, p, 3, cv::Scalar(0,255,0), 1);
    }

    // Ritorna l ultimo keypoint, solitamente però è uno.
    // Aggiungere dei controlli qui
    // Sto ritornando effettivamente il centro del blob? (NO)
    return p;

}

// Exploit pinhole camera model to compute X and Y, find Z in depth map
cv::Point3f MarkerTracker::findCoord3D(cv::Point2f point)
{
    // Sottoscrivo topic intrinsic parameters
    // Trovo i parametri che mi servono
    // calcolo X e Y

    // Sottoscrivo topic depth map OK ho depth_frame_

    // in (u,v) trovo valore di Z corrispondente
    unsigned short uZ = depth_frame_.at<unsigned short>(point.x,point.y);
    float Z = static_cast<float>(uZ);

    if (Z != 0)
        std::cout << "Z: " << Z << std::endl;

    // ritorno point3f (XYZ)
    return cv::Point3f(0.0,0.0,0.0);

}


void MarkerTracker::visualize()
{
    cv::imshow(IR_WINDOW, frame_);
    cv::imshow(OUTPUT_WINDOW, im_with_keypoints_);
    cv::imshow(DEPTH_WINDOW, depth_frame_);
    cv::waitKey(10);
}

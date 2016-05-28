/** Federico Vendramin 8 Maggio 2016
 *
 *  MarkerTracker.cpp
 *
 *
 *
 */

#include <MarkerTracker.hpp>

MarkerTracker::MarkerTracker()
    : it_(nh_)
{
    // Subscribe to input video feed and publish output video feed


    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    static const std::string IR_WINDOW = "IR Window";
    static const std::string DEPTH_WINDOW = "Depth Window";
    static const std::string OUTPUT_WINDOW = "Output Window";
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


void MarkerTracker::compute()
{
    static const std::string IR_WINDOW = "IR Window";
    static const std::string DEPTH_WINDOW = "Depth Window";
    static const std::string OUTPUT_WINDOW = "Output Window";
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
    cv::Mat im_with_keypoints;

    cv::threshold(img_source, img_black, 255, 255, cv::THRESH_BINARY);


    cv::drawKeypoints( img_black, keypoints, im_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
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


    for(int i = 0; i < keypoints.size(); ++i)
    {
        //std::cout << "disegnato keypoint!" << std::endl;
        cv::Point2f p = keypoints[i].pt;
        cv::circle(im_with_keypoints, p, 3, cv::Scalar(0,255,0), 1);
    }



    //std::cout << "OK!" << std::endl;
    cv::imshow(IR_WINDOW, img_source);
    cv::imshow(OUTPUT_WINDOW, im_with_keypoints);

    //depth_frame_.convertTo(depth_frame_, CV_8U, 1.0/256);
    //cv::equalizeHist(depth_frame_, depth_frame_);
    cv::imshow(DEPTH_WINDOW, depth_frame_);
    cv::waitKey(10);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());


}

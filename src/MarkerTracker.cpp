/** Federico Vendramin 8 Maggio 2016
 *
 *  MarkerTracker.cpp
 *
 *
 *
 */

#include <MarkerTracker.hpp>


MarkerTracker::MarkerTracker(std::string image_path, std::string depth_path)
    : it_(nh_)
{
    // Counting object with static variable, such that every camera has its image displayed in correct window

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(image_path, 5, &MarkerTracker::imageCb, this);
    depth_sub_ = it_.subscribe(depth_path, 5, &MarkerTracker::depthCb, this);
    info_sub_ = nh_.subscribe("/kinect2_head/depth/camera_info", 5, &MarkerTracker::cameraInfoCb, this);

    flag = false;
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // Initializes values
    f_x = 0.0;
    f_y = 0.0;
    c_x = 0.0;
    c_y = 0.0;
    X = 0.0;
    Y = 0.0;
    Z = 0.0;

    image_path_ = image_path;
    depth_path_ = depth_path;
    //    std::cout << Counter::howMany() << std::endl;
    //    currentID_ = Counter::howMany();

    //    IR_WINDOW = "IR Window" + std::to_string(currentID_);
    //    DEPTH_WINDOW = "Depth Window" + std::to_string(currentID_);
    //    OUTPUT_WINDOW = "Output Window" + std::to_string(currentID_);


    //    cv::namedWindow(IR_WINDOW);
    //    //cv::namedWindow(DEPTH_WINDOW);
    //    cv::namedWindow(OUTPUT_WINDOW);
}

MarkerTracker::MarkerTracker(const MarkerTracker &copia)
    : it_(nh_)
{
    image_sub_ = it_.subscribe(copia.image_path_, 5, &MarkerTracker::imageCb, this);
    depth_sub_ = it_.subscribe(copia.depth_path_, 5, &MarkerTracker::depthCb, this);
    info_sub_ = nh_.subscribe("/kinect2_head/depth/camera_info", 5, &MarkerTracker::cameraInfoCb, this);

    f_x = copia.f_x;
    f_y = copia.f_y;
    c_x = copia.c_x;
    c_y = copia.c_y;
    X = copia.X;
    Y = copia.Y;
    Z = copia.Z;

    frame_ = copia.frame_;
}

MarkerTracker::~MarkerTracker()
{
    //cv::destroyAllWindows();
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

void MarkerTracker::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
    if (!flag)
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
        flag = true;


    }

}


void MarkerTracker::setROI(int x, int y)
{
    //std::cout << "X , Y = " << x << " , " << y << std::endl;
    for (int j = 0; j < x; j++)
        for (int i = 0; i < y; i++)
            frame_.at<uchar>(i,j) = 0;

}

cv::Point2f MarkerTracker::findMarker()
{

    int threshold = 80;


    cv::Mat img_black;
    cv::Mat img_source = frame_;
    img_source.convertTo(img_source, CV_8UC1, 1.0/256);

    //cv::Mat img_prova = cv::imread("/home/federico/blob_detection.jpg");

    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 20.0f;
    params.minThreshold = 10; // 100
    params.maxThreshold = 255; //255

    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.blobColor = 255; //255
    params.filterByCircularity = false;

    params.filterByArea = true;
    params.minArea = .5; //1
    params.maxArea = 2.0; // 30

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
    // Sto ritornando effettivamente il centro del blob? (SI)
    return p;

}

// Exploit pinhole camera model to compute X and Y, find Z in depth map
cv::Point3f MarkerTracker::findCoord3D(cv::Point2f point)
{


    // in (u,v) trovo valore di Z corrispondente
    unsigned short uZ = depth_frame_.at<unsigned short>(point.x,point.y);
    Z = static_cast<float>(uZ)/1000;

    //if (Z != 0)
    // std::cout << "Z: " << Z << std::endl;


    // calcolo X e Y
    X = (point.x - c_x) * Z / f_x;
    Y = (point.y - c_y) * Z / f_y;


    // ritorno point3f (XYZ)
    return cv::Point3f(X,Y,Z);

}

bool MarkerTracker::hasIR()
{
    return (!frame_.empty());
}

void MarkerTracker::getFrame(cv::Mat &image)
{
    std::cout << "MAT: " << frame_ << std::endl;

    if (frame_.empty())
        ROS_INFO("Frame vuoto");

    image = frame_;
}



void MarkerTracker::visualize() //rimuovere
{
    cv::Mat out;
    frame_.convertTo(out, CV_8UC1, 1.0/256);
    //cv::equalizeHist(out,out);
    for( int y = 0; y < out.rows; y++ )
    { for( int x = 0; x < out.cols; x++ )
        { for( int c = 0; c < 3; c++ )
            {
                out.at<uchar>(y,x) = cv::saturate_cast<uchar>( 2.2*( out.at<uchar>(y,x))  );
            }
        }
    }

    //cv::imshow(IR_WINDOW, out);
    //cv::imshow(OUTPUT_WINDOW, im_with_keypoints_);
    //cv::imshow(DEPTH_WINDOW, depth_frame_);
    //cv::waitKey(30);
}

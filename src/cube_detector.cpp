#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



static const std::string OPENCV_WINDOW = "Image window";
int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";
int con_slider;
cv::Mat depth;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  geome
  cv::Mat ir_, depth_;
  std::vector<std::vector<cv::Point>> countours;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    /*image_sub_ = it_.subscribe("/kinect2_head/ir/image", 1,
                               &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/kinect2_head/depth/image", 1,
                               &ImageConverter::depthCb, this);
*/
    //cv::namedWindow(OPENCV_WINDOW);
    //cv::namedWindow("COLORI");
  }


  // returns sequence of squares detected on the image.
  // the sequence is stored in the specified memory storage
  static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point>>& countours )
  {
    countours.clear();
    cv::threshold(image, image, 150, 255, cv::THRESH_BINARY);
    //cv::Size kernelSize (2,2);
    //cv::Mat element = getStructuringElement (cv::MORPH_RECT, kernelSize, cv::Point(1,1));
    //morphologyEx(image, image, cv::MORPH_CLOSE, element );
    //cv::dilate(image, image, cv::Mat());
    cv::GaussianBlur(image, image, cv::Size(3,3), 0, 0);

    cv::findContours(image, countours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

  }


  // the function draws all the squares in the image
  static void drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point>>& countours )
  {
    //    for( size_t i = 0; i < squares.size(); i++ )
    //    {
    //      const cv::Point* p = &squares[i][0];
    //      int n = (int)squares[i].size();
    //      cv::polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 3, cv::LINE_AA);
    //    }

    cv::Mat dst = image.clone();
    //cv::cvtColor(image, dst, cv::COLOR_GRAY2BGR);
    //cv::GaussianBlur(image, image, cv::Size(3,3),0, 0);
    std::vector<cv::Point> approx;

    for( int i = 0; i < countours.size(); i++ )
    {

      cv::drawContours(dst, countours, i, cv::Scalar(255,255,255), CV_FILLED);
      /*
      if (countours[i].size() < 10)
        continue;
      std::cout << "Countour " << i << " size: " << countours[i].size() << std::endl;
      //RGB view
      if (i%5==0)
        cv::drawContours(dst, countours, i, cv::Scalar(255,0,0), CV_FILLED);
      if (i%5==1)
        cv::drawContours(dst, countours, i, cv::Scalar(0,255,0), CV_FILLED);
      if (i%5==2)
        cv::drawContours(dst, countours, i, cv::Scalar(0,0,255), CV_FILLED);
      if (i%5==3)
        cv::drawContours(dst, countours, i, cv::Scalar(0,128,255), CV_FILLED);
      if (i%5==4)
        cv::drawContours(dst, countours, i, cv::Scalar(128,255,0), CV_FILLED);
        */
    }

    //----------------BLOB DETECTOR-------->

    cv::SimpleBlobDetector::Params params;
    // Change thresholds
    params.filterByColor = 1;
    params.blobColor = 255;

    params.minThreshold = 150;
    params.maxThreshold = 255;

    // Filter by Area.
    params.filterByArea = 1;
    params.minArea = 100;
    params.maxArea = 2000;

    // Filter by Circularity
    params.filterByCircularity = 0;
    // Filter by Convexity
    params.filterByConvexity = 0;
    // Filter by Inertia
    params.filterByInertia = 0;



    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;

    detector->detect( dst, keypoints);

    //std::cout << "Keypoints found: " << keypoints.size() << std::endl;

    cv::Mat im_with_keypoints;
    cv::cvtColor(image,image, CV_GRAY2BGR);
    cv::drawKeypoints(image , keypoints, image, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );



    cv::Mat poly(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for(int i=0; i < countours.size(); ++i)
    {
      cv::approxPolyDP(cv::Mat(countours[i]), approx, arcLength(cv::Mat(countours[i]), true) * 0.03, true);
      if (approx.size() == 4)
      {
        //std::cout << "Drawing" << std::endl;
        cv::line(poly, approx.at(0), approx.at(1), cvScalar(255,0,0),1);
        cv::line(poly, approx.at(1), approx.at(2), cvScalar(255,0,0),1);
        cv::line(poly, approx.at(2), approx.at(3), cvScalar(255,0,0),1);
        cv::line(poly, approx.at(3), approx.at(0), cvScalar(255,0,0),1);
      }
    }
    cv::Mat blob(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::drawKeypoints(blob , keypoints, blob, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG );
    for(auto i = 0; i < keypoints.size(); ++i)
      std::cout << "Size: " << keypoints[i].size << std::endl;

    //cv::imshow("poly", poly);
    cv::imshow("BLOB", dst);
    cv::imshow("Original", image);
    cv::waitKey(30);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    cv::Mat image = cv_ptr->image;
    cv::Mat colorImage;
    image.convertTo(image, CV_8UC1, 1.0/256);
    cv::cvtColor(image, colorImage, cv::COLOR_GRAY2BGR);

    /*
    //HOUGH
    cv::threshold(image, image, 150, 255, cv::THRESH_BINARY);
    cv::GaussianBlur(image, image, cv::Size(3,3), 0, 0);
    std::vector<cv::Vec2f> lines;
    int votes = 12;
    HoughLines(image, lines, 1, CV_PI/180, votes, 0, 0);
    int count = 0;
    while(lines.size() < 4 && count<30)
    {
      count++;
      std::cout << "Size: " << lines.size() << std::endl;
      votes = votes - 2;

      lines.resize(0);
      HoughLines(image, lines, 1, CV_PI/180, votes, 0, 0);
    }
    count = 0;
    while(lines.size() > 9 && count < 30)
    {
      count++;
      std::cout << "Size: " << lines.size() << std::endl;
      votes = votes + 2;
      if(lines.size() > 20)
        votes = votes + 3;
      lines.resize(0);
      HoughLines(image, lines, 1, CV_PI/180, votes, 0, 0);
    }

    for( size_t i = 0; i < lines.size(); i++ )
    {
      float rho = lines[i][0], theta = lines[i][1];
      cv::Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( colorImage, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
    }
    // Update GUI Window

    cv::imshow(OPENCV_WINDOW, colorImage);
    cv::waitKey(30);
    */ // FINE HOUGH

    //CONTOURS

    std::vector<std::vector<cv::Point>> squares;
    //findSquares(image, squares);

    //drawSquares(image, squares);

  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
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

    depth = cv_ptr->image;
  }

  void callback(const sensor_msgs::ImageConstPtr& msg_ir, const sensor_msgs::ImageConstPtr& msg_depth)
  {

    cv_bridge::CvImagePtr cv_ptr_ir;
    cv_bridge::CvImagePtr cv_ptr_depth;
    try
    {
      cv_ptr_ir = cv_bridge::toCvCopy(msg_ir, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image = cv_ptr_ir->image;
    depth_ = cv_ptr_depth->image;

    image.convertTo(image, CV_8UC1, 1.0/256);

    findSquares(image, countours);

    cv::Mat dst = image.clone();
    cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
    //cv::GaussianBlur(image, image, cv::Size(3,3),0, 0);

    for( int i = 0; i < countours.size(); i++ )
    {

      cv::drawContours(dst, countours, i, cv::Scalar(255,255,255), CV_FILLED);
      /*
      if (countours[i].size() < 10)
        continue;
      std::cout << "Countour " << i << " size: " << countours[i].size() << std::endl;
      //RGB view
      if (i%5==0)
        cv::drawContours(dst, countours, i, cv::Scalar(255,0,0), CV_FILLED);
      if (i%5==1)
        cv::drawContours(dst, countours, i, cv::Scalar(0,255,0), CV_FILLED);
      if (i%5==2)
        cv::drawContours(dst, countours, i, cv::Scalar(0,0,255), CV_FILLED);
      if (i%5==3)
        cv::drawContours(dst, countours, i, cv::Scalar(0,128,255), CV_FILLED);
      if (i%5==4)
        cv::drawContours(dst, countours, i, cv::Scalar(128,255,0), CV_FILLED);
        */
    }

    //----------------BLOB DETECTOR-------->

    cv::SimpleBlobDetector::Params params;
    // Change thresholds
    params.filterByColor = 1;
    params.blobColor = 255;
    params.minThreshold = 150;
    params.maxThreshold = 255;
    // Filter by Area.
    params.filterByArea = 1;
    params.minArea = 10;
    params.maxArea = 2000;
    params.filterByCircularity = 0;
    params.filterByConvexity = 0;
    params.filterByInertia = 0;

    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;

    detector->detect( dst, keypoints);

    cv::Mat im_with_keypoints;
    cv::cvtColor(image,image, CV_GRAY2BGR);
    cv::drawKeypoints(image , keypoints, image, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    cv::Mat blob(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::drawKeypoints(dst , keypoints, dst, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    //for(auto i = 0; i < keypoints.size(); ++i)
    //  std::cout << "Size: " << keypoints[i].size << std::endl;


    double radius, medianDepth;
    int centerX, centerY;
    std::vector<double> depthValues;



    //Now use radius to find depth
    if (keypoints.size() == 1)
    {
      depthValues.resize(0);
      radius = keypoints[0].size;
      centerX = keypoints[0].pt.x;
      centerY = keypoints[0].pt.y;

      for(int i = centerX - radius; i < centerX + radius; ++i) {
        for(int j = centerY - radius; j < centerY + radius; ++j) {
          //Scan all points in rectangle
          if (dst.at<uchar>(j,i) == 255) {
            continue;
          } else if (pow(centerX - i, 2) + pow(centerY - j, 2) <= radius) {
            double depthTemp = static_cast<double>(depth_.at<unsigned short>(j,i)) / 1000;
            if (depthTemp != 0) {
              depthValues.push_back(depthTemp);
              cv::circle(image, cv::Point(i,j), 1, cv::Scalar(0, 255, 0));
              //image.at<cv::Vec3d>(j,i) = cv::Vec3d(0, 255, 0);
            }
          }
        } // nested for
      } // for

      std::sort(depthValues.begin(), depthValues.end());
      if (depthValues.size() % 2 == 0)
        medianDepth = depthValues[depthValues.size()/2];
      else
        medianDepth = depthValues[floor(depthValues.size()/2)];

      //DEBUG numbers
      //for(int i = 0; i < depthValues.size(); ++i)
      //  std::cout << depthValues[i] << "  ";
      //std::cout << std::endl;

      std::vector<double> refinedValues;

      std::cout << "minValue: " << depthValues[0] << " avgValue: " << medianDepth << " maxValue: " << depthValues[depthValues.size()-1] << std::endl;

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

      } // Refinement


      //Publish msg + frame to view with RVIZ


    } // if keyp size

    //cv::imshow("poly", poly);
    cv::imshow("BLOB", dst);
    cv::imshow("Original", image);
    //cv::imshow("depth", depth_);
    cv::waitKey(30);

  } // End Callback()
}; // End class Image Converter


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;

  ImageConverter ic;

  message_filters::Subscriber<sensor_msgs::Image> ir_sub(nh, "/kinect2_head/ir/image", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2_head/depth/image", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ir_sub, depth_sub);
  sync.registerCallback(boost::bind(&ImageConverter::callback, ic,  _1, _2));

  ros::spin();
  return 0;
}

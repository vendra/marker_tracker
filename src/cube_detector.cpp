#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/me/vpMeLine.h>


static const std::string OPENCV_WINDOW = "Image window";
int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2_head/ir/image", 1,
                               &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  // helper function:
  // finds a cosine of angle between vectors
  // from pt0->pt1 and from pt0->pt2
  static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
  {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
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
    cv::GaussianBlur(image, image, cv::Size(3,3),0, 0);

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
    cv::cvtColor(image, dst, cv::COLOR_GRAY2BGR);
    //cv::GaussianBlur(image, image, cv::Size(3,3),0, 0);
    std::vector<cv::Point> approx;
    /*
    for( int i = 0; i < countours.size(); i++ )
    {
      drawContours(dst, countours, i, cv::Scalar(255,0,0), 2, 8, hierarchy, 0, Point() );
    } */


    for(int i=0; i < countours.size(); ++i)
    {
      cv::approxPolyDP(cv::Mat(countours[i]), approx, arcLength(cv::Mat(countours[i]), true) * 0.03, true);
      if (approx.size() == 4)
      {
        //std::cout << "Drawing" << std::endl;
        line(dst, approx.at(0), approx.at(1), cvScalar(255,0,0),2);
        line(dst, approx.at(1), approx.at(2), cvScalar(255,0,0),2);
        line(dst, approx.at(2), approx.at(3), cvScalar(255,0,0),2);
        line(dst, approx.at(3), approx.at(0), cvScalar(255,0,0),2);
      }
    }

    cv::imshow("COLORI", dst);
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

    //HOUGH
    std::vector<cv::Vec2f> lines;
    HoughLines(image, lines, 1, CV_PI/180, 100, 0, 0 );

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
      line( colorImage, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }

    //CONTOURS

    std::vector<std::vector<cv::Point>> squares;
    //findSquares(image, squares);

    //drawSquares(image, squares);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, colorImage);
    cv::waitKey(30);

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

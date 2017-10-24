/** Federico Vendramin 8th May 2016
 *
 *  MarkerTracker.hpp
 *
 */

#ifndef MARKERTRACKER_H_
#define MARKERTRACKER_H_

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>

#include <math.h>


class MarkerTracker
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    cv::Mat frame_, depth_frame_, im_with_keypoints_;
    cv::Mat cameraMatrix, distCoeffs;
    float X,Y,Z;
    std::vector<cv::KeyPoint> keypoints_;
    std::string image_path_, depth_path_;
    cv::SimpleBlobDetector::Params params;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    std::vector<cv::Point2f> maskPoints;

    cv::Mat blobImage;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::KeyPoint> keypoints; // also have keypoints_ use only one
    std::vector<double> depthValues;
    std::vector<cv::Point> depthPoints;
    std::vector<double> refinedValues;
    double radius, medianDepth;
    int centerX, centerY;
    bool newFrame;

    //Private Functions

    void applyMask(); 

    void findDepthValues();

    void findMedianDepth();

    void refineMedianDepth();

public:
    MarkerTracker(std::string image_path, std::string depth_path,
                  std::string param_path, std::string calib_path);

    //Loads class parameter from .YAML file specified as parameter
    bool readInputParams(std::string path);

    //Loads intrinsic camera parameters from .YAML specified as parameter
    bool readCameraParams(std::string path);

    bool newFrameArrived();

    //Callback reading ir frame
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    //Callback reading depth frame
    void depthCb(const sensor_msgs::ImageConstPtr& msg); //useless now

    //Set the mask for the IR frame to avoid detecting false positive, i.e. ir reflection, other cameras
    void setMask(const std::vector<cv::Point2f> mask);    

    //Detection of the IR POINT marker in the image
    //Returns a point (u,v) containing coordinates of the marker
    cv::Point2f findMarker();

    //Detection of the IR CUBIC marker in the image
    //Returns a point (u,v) containing coordinates of the marker
    cv::KeyPoint detectMarker();

    //private
    void findMarkerContours( const cv::Mat& image, std::vector<std::vector<cv::Point>>& contours );

    //Backprojection of the 2D point, returns 3D point with respect to the camera frame
    cv::Point3f findCoord3D(cv::Point2f point);

    //New for cube depth
    cv::Point3f findCoord3D(cv::KeyPoint point);

    void findMarkerDepth(const cv::KeyPoint markerKeypoint);

    //Returns true if an IR frame has been published and read
    bool hasIR();

    //Returns true if a Depth frame has been published and read
    bool hasDepth();

    //Pass the latest IR frame
    void getIRFrame(cv::Mat& image);

    //Pass the latest Depth frame
    void getDepthFrame(cv::Mat& depth);

    //Pass the latest output(RGB) frame with the detected points highlighted
    void getOutputFrame(cv::Mat& out);

};

#endif // MARKERTRACKER_H

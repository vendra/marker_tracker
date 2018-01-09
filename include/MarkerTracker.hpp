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

public:

    /*
     * Constructor, Specify string paths for:
     * IR imagetopic, Depth image topic,
     * Path to detection parameters and intrinsic camera parameters
     */
    MarkerTracker(std::string image_path, std::string depth_path,
                  std::string param_path, std::string calib_path);

    /*
     * IR image callback
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /*
     * Depth image callback
     */
    void depthCb(const sensor_msgs::ImageConstPtr& msg);

    /*
     * Parses parameters from specified file in the path
     */
    bool readInputParams(std::string path);

    /*
     * Set the specified vector of points as the new mask
     */
    void setMask(const std::vector<cv::Point2f> mask);

    /*
     * Legacy detection, used for the spheric marker detection
     * Returns a cv::Point2f with the x,y coordinates of the marker on the image
     */
    cv::Point2f findMarker();

    /*
     * New detection for the cubic marker
     * Returns the cv::Keypoint containing the position x,y of the marker on the image
     * and its size
     */
    cv::KeyPoint detectMarker();

    /*
     * Returns the 3D backprojection of the specified 2D Point
     * Uses intrinsics camera parameters and exploit pinhole model
     */
    cv::Point3f findCoord3D(cv::Point2f point);

    /*
     * Returns the 3D backprojection of the specified 2D Keypoint
     * Uses intrinsics camera parameters and exploit pinhole model
     */
    cv::Point3f findCoord3D(cv::KeyPoint point);

    /*
     * Computes the estimated depth of the specified Keypoint area
     * Assumes the specified keypoint is a valid detection
     */
    void findMarkerDepth(const cv::KeyPoint markerKeypoint);

    /*
     * Returns true if an IR frame has been published and read
     */
    bool hasIR();

    /*
     * Returns true if a Depth frame has been published and read
     */
    bool hasDepth();

    /*
     * Pass the latest IR image frame
     */
    void getIRFrame(cv::Mat& image);

    /*
     * Pass the latest Depth image frame
     */
    void getDepthFrame(cv::Mat& depth);

    /*
     * Pass the latest output(RGB) frame with the detected points drawn
     */
    void getOutputFrame(cv::Mat& out);



private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    cv::Mat cameraMatrix, distCoeffs; // Camera parameters
    cv::Mat frame_, depth_frame_, im_with_keypoints_, blobImage; //Images
    std::string image_path_, depth_path_; // Topic paths for IR and Depth frames
    cv::SimpleBlobDetector::Params params;
    cv::Ptr<cv::SimpleBlobDetector> detector; // OpenCV blog detector

    std::vector<cv::Point2f> maskPoints; // Clicked mask points
    std::vector<std::vector<cv::Point>> contours; // Contours of the detected marker
    std::vector<cv::KeyPoint> keypoints; // Keypoints of detected marker
    std::vector<double> depthValues; // Selected Depth Values
    std::vector<cv::Point> depthPoints; //Candidate Depth Values
    std::vector<double> refinedValues; // Refined Depth Values
    double medianDepth; // Estimated depth
    int centerX, centerY; // Blob center (x,y) coordinates and radius (size)
    double radius;
    double X,Y,Z; //3D point coordinates


    //Private Functions

    /*
     * Parses intrisinc camera parameters from specified path
     */
    bool readCameraParams(std::string path);

    /*
     * Apply user masking to the IR frame to avoid wrong detections due to reflections
     */
    void applyMask(); 

    /*
     * Computes the Depth of the detected marker. Filter out points lying on the marker's edges.
     */
    void findDepthValues();

    /*
     * Computes the median point between the median depth points of the marker
     */
    void findMedianDepth();

    /*
     * Refines the median point by removing points too far from the previous median
     * This can be avoided since the medianDepth is already good enough.
     */
    void refineMedianDepth();

    /*
     * Finds contours of the IR image
     */
    void findMarkerContours( const cv::Mat& image, std::vector<std::vector<cv::Point>>& contours );

};

#endif // MARKERTRACKER_H


/*
 *  Copyright (c) 2016-, VENDRAMIN FEDERICO <vendra22@gmail.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <marker_tracker_node.h>


static const std::string IR_WINDOW = "IR Window";
static const std::string DEPTH_WINDOW = "Depth Window";
static const std::string OUTPUT_WINDOW = "Output Window";
int threshold = 14000;


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
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2_head/ir_rect/image", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(IR_WINDOW);
    cv::namedWindow(DEPTH_WINDOW);
    cv::namedWindow(OUTPUT_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyAllWindows();
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

    cv::Mat img_threshold;
    cv::Mat img_source = cv_ptr->image;
    img_source.convertTo(img_source, CV_32F);
    cv::threshold(img_source, img_threshold, threshold, 2550, cv::THRESH_BINARY);
    
    cv::SimpleBlobDetector detector;
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(img_source, keypoints);
    cv::Mat im_with_keypoints;
    cv::drawKeypoints( img_source, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    cv::imshow(IR_WINDOW, cv_ptr->image);
    cv::imshow(OUTPUT_WINDOW, im_with_keypoints);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int
main (int argc , char ** argv)
{
  ros::init(argc, argv, "Marker_tracker_node"); // Node name. In ROS graph (e.g rqt_graph): /class_template
  auto node_handle = ros::NodeHandle("~"); // This node handle make the topics and parameters be
  // relative to the node, i.e. /class_template/<something>

  /*
    auto node = unipd::euroc::ClassTemplate(node_handle); // Create object
    if (not node.initialize())
      return 1;
  */
  cv::namedWindow("IR", cv::WINDOW_AUTOSIZE);

  ImageConverter ic;

  cv::SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 50.0f;
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;
  params.minArea = 2.0f;
  params.maxArea = 50.0f;

  cv::SimpleBlobDetector detector(params);


  cv::createTrackbar("Threshold", OUTPUT_WINDOW, &threshold, 65000);


  //node.spin(); // Let the node run until it finishes
  ros::spin();
  return 0;
}

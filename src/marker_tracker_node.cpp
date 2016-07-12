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
#include <MarkerTracker.hpp>


int main (int argc , char ** argv)
{
    ros::init(argc, argv, "Marker_tracker_node"); // Node name. In ROS graph (e.g rqt_graph): /class_template
    //auto node_handle = ros::NodeHandle("~"); // This node handle make the topics and parameters be
    ros::NodeHandle node_handle;

    // relative to the node, i.e. /class_template/<something>

    /*
    auto node = unipd::euroc::ClassTemplate(node_handle); // Create object
    if (not node.initialize())
      return 1;
  */
    cv::namedWindow("IR", cv::WINDOW_AUTOSIZE);

    MarkerTracker mt;
    ros::Subscriber image_sub = node_handle.subscribe("/kinect2_head/ir/image", 5,&MarkerTracker::imageCb, &mt);
    ros::Subscriber depth_sub = node_handle.subscribe("/kinect2_head/depth/image", 5, &MarkerTracker::depthCb, &mt);
    ros::spinOnce();
    ros::Duration(2.0).sleep();
    ros::spinOnce();

    //cv::createTrackbar("Threshold", OUTPUT_WINDOW, &threshold, 100);

    while(node_handle.ok())
    {
        cv::Point2f a = mt.findMarker();
        cv::Point3f b = mt.findCoord3D(a);
        mt.visualize();

        //Let the node run until it finishes
        ros::spinOnce();
    }
    return 0;
}

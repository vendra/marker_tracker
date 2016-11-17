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

#include <marker_tracker_node.hpp>
#include <MarkerTracker.hpp>


int main (int argc , char ** argv)
{
    ros::init(argc, argv, "Marker_tracker_node"); // Node name. In ROS graph (e.g rqt_graph): /class_template
    //auto nh = ros::NodeHandle("~"); // This node handle make the topics and parameters be
    ros::NodeHandle nh;

    // relative to the node, i.e. /class_template/<something>

    /*
    auto node = unipd::euroc::ClassTemplate(nh); // Create object
    if (not node.initialize())
      return 1;
  */
    //cv::namedWindow("IR", cv::WINDOW_AUTOSIZE);
    // Conviene sistemare e creare un array di mt. cosi da evitare di creare a mano tutto
    //Nel setup faccio un pushback ogni volta, da sistemare
    MarkerTracker mt1("/kinect2_10/ir/image", "/kinect2_10/depth/image");
    MarkerTracker mt2("/kinect2_12/ir/image", "/kinect2_12/depth/image");
    MarkerTracker mt3("/kinect2_13/ir/image", "/kinect2_13/depth/image");
    MarkerTracker mt4("/kinect2_16/ir/image", "/kinect2_16/depth/image");


    ros::spinOnce();
    ros::Duration(2.0).sleep();
    ros::spinOnce();

    //cv::createTrackbar("Threshold", OUTPUT_WINDOW, &threshold, 100);

    while(nh.ok())
    {
        cv::Point2f a1 = mt1.findMarker();
        //std::cout << "Coordinate 1 centro marker X: " << a1.x << " Y: "<< a1.y << std::endl;
        cv::Point3f b1 = mt1.findCoord3D(a1);
        //std::cout << "---------------------------------------------------------------" << std::endl;
        cv::Point2f a2 = mt2.findMarker();
        //std::cout << "Coordinate 2 centro marker X: " << a2.x << " Y: "<< a2.y << std::endl;
        cv::Point3f b2 = mt2.findCoord3D(a2);
        //std::cout << "---------------------------------------------------------------" << std::endl;
        cv::Point2f a3 = mt3.findMarker();
        //std::cout << "Coordinate 3 centro marker X: " << a3.x << " Y: "<< a3.y << std::endl;
        cv::Point3f b3 = mt3.findCoord3D(a3);
        //std::cout << "---------------------------------------------------------------" << std::endl;
        cv::Point2f a4 = mt4.findMarker();
        //std::cout << "Coordinate 4 centro marker X: " << a4.x << " Y: "<< a4.y << std::endl;
        cv::Point3f b4 = mt4.findCoord3D(a4);


        //Spostare la visualizzazione fuori dalla classe che è meglio!!
        mt1.visualize();
        mt2.visualize();
        mt3.visualize();
        mt4.visualize();

        //std::cout << "Coordinate 3D X: " << b.x << " Y: " << b.y << " Z: " << b.z << std::endl;
        //Let the node run until it finishes
        ros::spinOnce();
    }
    return 0;
}

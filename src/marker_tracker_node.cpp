/*
 *  Copyright (c) 2016-, VENDRAMIN FEDERICO <federico.vendramin@gmail.com>
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
    ros::NodeHandle nh;

    // vector of MarkerTracker, one for each kinect
    std::vector<std::shared_ptr<MarkerTracker>> mt;

    mt.clear();
    mt.push_back(std::make_shared<MarkerTracker>("/kinect2_10/ir/image", "/kinect2_10/depth/image"));
    mt.push_back(std::make_shared<MarkerTracker>("/kinect2_12/ir/image", "/kinect2_12/depth/image"));
    mt.push_back(std::make_shared<MarkerTracker>("/kinect2_13/ir/image", "/kinect2_13/depth/image"));
    mt.push_back(std::make_shared<MarkerTracker>("/kinect2_16/ir/image", "/kinect2_16/depth/image"));

    std::cout << "Object count: " << mt.size() << std::endl;
    std::vector<std::string> IR_WINDOWS;
    std::vector<std::string> OUT_WINDOWS;

    IR_WINDOWS.clear();
    IR_WINDOWS.push_back("IR 1");
    IR_WINDOWS.push_back("IR 2");
    IR_WINDOWS.push_back("IR 3");
    IR_WINDOWS.push_back("IR 4");

    OUT_WINDOWS.clear();
    OUT_WINDOWS.push_back("OUT 1");
    OUT_WINDOWS.push_back("OUT 2");
    OUT_WINDOWS.push_back("OUT 3");
    OUT_WINDOWS.push_back("OUT 4");

    cv::startWindowThread();
    cv::namedWindow("Setup");

    cv::Mat frame, depth, out;

    // wait for images to be published
    ros::spinOnce();
    ros::Duration(2.0).sleep();
    ros::spinOnce();

    int x_slider = 0;
    int y_slider = 0;

    ROS_INFO("Press q to confirm and proceed");

    char c;
    for (auto i = 0; i < mt.size(); ++i)
    {
        x_slider = 0;
        y_slider = 0;
        cv::createTrackbar("X", "Setup", &x_slider, 1024);
        cv::createTrackbar("Y", "Setup", &y_slider, 512);
        while(true)
        {
            ros::spinOnce();
            mt[i]->getIRFrame(frame);
            mt[i]->setROI(x_slider,y_slider);

            frame.convertTo(frame,CV_8UC1, 1.0/256);
            if (!frame.empty())
            {
                for( int y = 0; y < frame.rows; y++ )
                { for( int x = 0; x < frame.cols; x++ )
                    { for( int c = 0; c < 3; c++ )
                        {
                            frame.at<uchar>(y,x) = cv::saturate_cast<uchar>( 2.2*( frame.at<uchar>(y,x))  );
                        }
                    }
                }
                cv::imshow("Setup", frame);
            }


            c = cv::waitKey(30);
            if (c == 'q')
                break;
        }
    }

    cv::destroyWindow("Setup");
    cv::waitKey(30);
    ROS_INFO("Setup Completed");

    // Create new Windows
    for (int i = 0; i < IR_WINDOWS.size(); i++)
        cv::namedWindow(OUT_WINDOWS[i]);

    while(nh.ok())
    {

        for (auto i = 0; i < mt.size(); ++i)
        {
            cv::Point2f a1 = mt[i]->findMarker();
            //std::cout << "Coordinate 1 centro marker X: " << a1.x << " Y: "<< a1.y << std::endl;
            cv::Point3f b1 = mt[0]->findCoord3D(a1);
            //std::cout << "---------------------------------------------------------------" << std::endl;
        }

        // let the user see
        for(int i = 0; i< IR_WINDOWS.size(); i++)
        {
            mt[i]->getOutputFrame(out);
            imshow(OUT_WINDOWS[i],out);
        }
        cv::waitKey(30);

        //std::cout << "Coordinate 3D X: " << b.x << " Y: " << b.y << " Z: " << b.z << std::endl;

        //Let the node run until it finishes
        ros::spinOnce();
    }
    return 0;
}

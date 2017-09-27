/*  Elaborazione dei dati tridimensionali - Università degli studi di Padova
 *
 *  Copyright (c) 2016- VENDRAMIN FEDERICO <federico.vendramin@gmail.com>
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
#include <ros/console.h>

cv::Mat depth_frame;
cv_bridge::CvImagePtr cv_ptr;

void depthCb(const sensor_msgs::ImageConstPtr&  msg)
{
    try
    {
        //Decode
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		depth_frame = cv_ptr->image;
        //cv::imshow("view", cv_bridge::toCvShare(msg, "16UC1")->image);
		//cv::waitKey(30);
		//REPUBLISH
		//sensor_msgs::ImagePtr msg_to_send = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_ptr->image).toImageMsg();
		//depth_pub.publish(msg_to_send);
		
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

void mouseClick(int event, int x, int y, int flags, void* maskPoints)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "Set mask in position (" << x << ", " << y << ")" << std::endl;
          std::vector<cv::Point2f> *maskPointPtr = static_cast<std::vector<cv::Point2f> *>(maskPoints);
          cv::Point2f pt = cv::Point2f(x,y);
          (*maskPointPtr).push_back(pt);
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          std::cout << "Reset mask" << std::endl;
          std::vector<cv::Point2f> *maskPointPtr = static_cast<std::vector<cv::Point2f> *>(maskPoints);          
          (*maskPointPtr).resize(0);
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
          //std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;

     }
}



int main (int argc , char* argv[])
{
    ros::init(argc, argv, "Marker_tracker_node"); // Node name. In ROS graph (e.g rqt_graph): /class_template
    ros::NodeHandle nh("~");
    
    //Read ID
    std::string id = argv[1];
    nh.getParam("id", id); 
    //std::cout << "ID read: " << argv[1] << std::endl; //Need to to some more controls here

    //ROS_ERROR(argv[1]);

    //For now all nodes uses the same YAML but its easy to replace using files with
    //id.yaml and reading those instead 
    std::string param_path = ros::package::getPath("marker_tracker")+"/parameters.yaml";
    std::string calib_path = ros::package::getPath("marker_tracker")+"/param/"+id+".yaml";
    std::cout << "param path: " << param_path << std::endl;
    std::cout << "calib path: " << calib_path << std::endl;

    MarkerTracker tracker("/"+id+"/ir/image", "/"+id+"/depth/image",
                          param_path, calib_path);
    
    ros::Subscriber sub = nh.subscribe("/"+id+"/depth/image", 100, depthCb);
    
    // wait for images to be published
    ros::spinOnce();
    ros::Duration(2.0).sleep();
    ros::spinOnce();

    cv::Mat frame, depth, out;
    int x_slider = 0;
    int y_slider = 0;
    std::vector<cv::Point2f> maskPoints;

    
    cv::startWindowThread();
    cv::namedWindow(id+"Setup");
    cv::setMouseCallback(id+"Setup", mouseClick, &maskPoints);

    ROS_INFO("Press q to confirm and proceed");
    char c;

    x_slider = 0;
    y_slider = 0;
    cv::createTrackbar("X", id+"Setup", &x_slider, 1024);
    cv::createTrackbar("Y", id+"Setup", &y_slider, 424);

    bool exit_key_pressed = false;
    while (!exit_key_pressed)
    {
        ros::spinOnce();
        //cv::Mat frame_depth;
        tracker.getIRFrame(frame);
        //tracker.getDepthFrame(frame_depth);
        //tracker.setROI(x_slider,y_slider);
        tracker.findMarker();
        tracker.getOutputFrame(out);
        
        //ROI setup visulization
        /*if (!out.empty())
        {
            for( int y = 0; y < y_slider; y++ )
                for( int x = 0; x < x_slider; x++  )
                    out.at<uchar>(y,x) = cv::saturate_cast<uchar>( 2.2*( out.at<uchar>(y,x))  );
        }*/

        //Only visualization!!!!!!!
        //Apply black mask
        for(int i=0; i < maskPoints.size(); ++i)
            cv::circle(out, maskPoints[i], 3, cv::Scalar(0,0,0), -1);

        cv::imshow(id+"Setup", out);
        c = cv::waitKey(30);
        if (c == 'q')
            exit_key_pressed = true;
        if (c== '.') 
            tracker.readInputParams(param_path);
    }

    tracker.setMask(maskPoints);
    

    cv::destroyWindow(id+"Setup");
    cv::waitKey(30);
    ROS_INFO("Setup Completed");

    // Create new Windows
    cv::namedWindow(id+"Output");

    while(nh.ok())
    {   
        tracker.setDepthFrame(depth_frame);
        cv::Point2f a1 = tracker.findMarker();
        std::cout << "Coordinate 1 centro marker X: " << a1.x << " Y: "<< a1.y << std::endl;
        cv::Point3f b1 = tracker.findCoord3D(a1);
        //std::cout << "---------------------------------------------------------------" << std::endl;

        // let the user see

        tracker.getOutputFrame(out);
        imshow(id+"Output",out);
        cv::waitKey(30);

        std::cout << "Coordinate 3D X: " << b1.x << " Y: " << b1.y << " Z: " << b1.z << std::endl;

        //Let the node run until it finishes
        ros::spinOnce();
        c = cv::waitKey(30);
        if (c == 'q')
            break;
    }

    return 0;
}

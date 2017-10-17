/*  Elaborazione dei dati tridimensionali - Università degli studi di Padova
 *
 *  Copyright (c) 2017- VENDRAMIN FEDERICO <federico.vendramin@gmail.com>
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
 *  DISCLAIMED. IN NO EVENT SHALL Federico Vendramin BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

tf::StampedTransform k2transf;
tf::StampedTransform k3transf;
//tf::TransformListener listenerCb;

void positionCb(const geometry_msgs::PointStampedConstPtr pos1, 
                const geometry_msgs::PointStampedConstPtr pos2,
                const geometry_msgs::PointStampedConstPtr pos3) 
{
    //std::cout << "SEQS:: " << pos1->header.seq << " " << pos2->header.seq <<
    //" " << pos3->header.seq << std::endl;

    tf::Vector3 point2, point3;
    point2.setX(pos2->point.x);
    point2.setY(pos2->point.y);
    point2.setZ(pos2->point.z);

    point3.setX(pos3->point.x);
    point3.setY(pos3->point.y);
    point3.setZ(pos3->point.z);

    tf::Vector3 transfPoint2 = k2transf * point2;
    tf::Vector3 transfPoint3 = k3transf * point3;


    std::cout << "NORMAL tf" << std::endl;
    std::cout << "X: " << pos1->point.x << " " << transfPoint2.getX() << 
                 " " << transfPoint3.getX() << std::endl;
    std::cout << "Y: " << pos1->point.y << " " << transfPoint2.getY() << 
                 " " << transfPoint3.getY() << std::endl;
    std::cout << "Z: " << pos1->point.z << " " << transfPoint2.getZ() << 
                 " " << transfPoint3.getZ() << std::endl;
    std::cout << std::endl;

    //geometry_msgs::PointStamped out;
    //listenerCb.transformPoint("/k1_ir_frame", *pos2, out);

}

int main (int argc , char* argv[])
{
    ros::init(argc, argv, "Master_node");
    ros::NodeHandle nh("~");

    message_filters::Subscriber<geometry_msgs::PointStamped> k1_sub(nh, "/k1/position", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> k2_sub(nh, "/k2/position", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> k3_sub(nh, "/k3/position", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped,
                                                            geometry_msgs::PointStamped, 
                                                            geometry_msgs::PointStamped> PosSyncPolicy;
    
    message_filters::Synchronizer<PosSyncPolicy> sync(PosSyncPolicy(20), k1_sub, k2_sub, k3_sub);
    sync.registerCallback(boost::bind(&positionCb, _1, _2, _3));


    //Lookup transform from kx to master k1 for example
    tf::TransformListener listener;

    while(nh.ok()) {
        try {
            listener.lookupTransform("k1", "k2",
                                      ros::Time(0), k2transf);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            std::cout << "Transform K2 to K1 OK!" << std::endl;
            break;
    }

    while(nh.ok()) {
        try {
            listener.lookupTransform("k1", "k3",
                                      ros::Time(0), k3transf);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            std::cout << "Transform K3 to K1 OK!" << std::endl;
            break;
    }   

    std::cout << "K2 to K1: " << k2transf.getRotation() << std::endl;
    //Do interpolation of some kind / median

    ros::spin();

    return 0;
}
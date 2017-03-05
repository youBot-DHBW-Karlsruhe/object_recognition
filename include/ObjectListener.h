#ifndef OBJECTKLISTENER_H
#define OBJECTLISTENER_H

#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>

namespace cleaner {

class ObjectListener {

private:

    std::string mapFrameId_;
    std::string objFramePrefix_;
    tf::TransformListener tfListener_;
    ros::Subscriber subs_;
    ros::Publisher pub_;
    std_msgs::Float32MultiArray coordinates;

public:

    ObjectListener(ros::NodeHandle nh) {
        mapFrameId_ = "/map";
        objFramePrefix_ = "object";
        coordinates = std_msgs::Float32MultiArray();
        subs_ = nh.subscribe("/objectsStamped", 1, &ObjectListener::objectsDetectedCallback, this);
        pub_ = nh.advertise<std_msgs::Float32MultiArray>("object_position", 1);
        //ros::spin();
    }

    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg){

        //convert data
        if(msg->objects.data.size())
                {
                    for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
                    {
                        // get data
                        int id = (int)msg->objects.data[i];
                        std::stringstream ss;
                        ss << "object_" << id;
                        std::string objectFrameId = ss.str();
                        ROS_INFO("object_%d", id);
                        tf::StampedTransform pose;
                        tf::StampedTransform poseCam;
                        try
                        {
                            // Get transformation from "object_#" frame to target frame "map"
                            // The timestamp matches the one sent over TF

                            tfListener_.lookupTransform(mapFrameId_, objectFrameId, msg->header.stamp, pose);
                            tfListener_.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
                        }
                        catch(tf::TransformException & ex)
                        {
                            ROS_WARN("%s",ex.what());
                            continue;
                        }

                        ROS_INFO("lebt noch 1");
                        // Here "pose" is the position of the object "id" in "/map" frame.
                        /*ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                                id, mapFrameId_.c_str(),
                                pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                                pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
                        ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                                id, msg->header.frame_id.c_str(),
                                poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
                                poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());*/

                        ROS_INFO("%f", pose.getOrigin().x());
                        double x = pose.getOrigin().x();
                        float fx = (float) x;
                        coordinates.data.push_back(fx);
                        //coordinates.data[0] = fx;
                        //This causes process to die
                        /*coordinates.data[0] = pose.getOrigin().x();
                        coordinates.data[1] = pose.getOrigin().y();
                        coordinates.data[2] = pose.getOrigin().z();
                        coordinates.data[3] = pose.getRotation().x();
                        coordinates.data[4] = pose.getRotation().y();
                        coordinates.data[5] = pose.getRotation().z();
                        coordinates.data[6] = pose.getRotation().w();*/

                        //TODO aggregate received data to ensure quality and useful grab positions

                        //publish data
                        //publishPosition();


                    }
        }

    }

    void publishPosition() {
        pub_.publish(coordinates);
    }

};

}

#endif

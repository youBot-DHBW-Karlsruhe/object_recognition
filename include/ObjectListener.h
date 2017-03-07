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

                        coordinates.data.push_back(pose.getOrigin().x());
                        coordinates.data.push_back(pose.getOrigin().y());
                        coordinates.data.push_back(pose.getOrigin().z());
                        coordinates.data.push_back(pose.getRotation().x());
                        coordinates.data.push_back(pose.getRotation().y());
                        coordinates.data.push_back(pose.getRotation().z());
                        coordinates.data.push_back(pose.getRotation().w());

                        //TODO aggregate received data to ensure quality and useful grab positions

                        //publish data
                        publishPosition();


                    }
        }

    }

    void publishPosition() {
        pub_.publish(coordinates);
        coordinates.data.clear();
    }

};

}

#endif

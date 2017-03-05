#ifndef OBJECTKLISTENER_H
#define OBJECTLISTENER_H

#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <object_recognition/ObjectPosition.h>

namespace cleaner {

class ObjectListener {

private:

    std::string mapFrameId_;
    std::string objFramePrefix_;
    tf::TransformListener tfListener_;
    ros::Subscriber subs_;
    ros::Publisher pub_;
    object_recognition::ObjectPosition position_;

public:

    ObjectListener(ros::NodeHandle nh) {
        mapFrameId_ = "/map";
        objFramePrefix_ = "object";
        subs_ = nh.subscribe("/objectsStamped", 1, &ObjectListener::objectsDetectedCallback, this);
        pub_ = nh.advertise<object_recognition::ObjectPosition>("object_position", 1);
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
                        position_.object_id = id;
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

                        object_recognition::ObjectPosition position;
                        position.pose.position.x = pose.getOrigin().x();
                        position.pose.position.y = pose.getOrigin().y();
                        position.pose.position.z = pose.getOrigin().z();
                        position.pose.orientation.x = pose.getRotation().x();
                        position.pose.orientation.x = pose.getRotation().y();
                        position.pose.orientation.x = pose.getRotation().z();
                        position.pose.orientation.x = pose.getRotation().w();

                        //TODO aggregate received data to ensure quality and useful grab positions

                        //publish data
                        publishPosition(position);


                    }
        }

    }

    void publishPosition(object_recognition::ObjectPosition position) {
        pub_.publish(position);
    }

};

}

#endif

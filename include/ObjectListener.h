#ifndef OBJECTKLISTENER_H
#define OBJECTLISTENER_H

#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <object_recognition/ObjectPosition.h>

namespace cleaner {

typedef struct {
    std::vector<object_recognition::ObjectPosition> positions;
    std::string objectId;
} ObjectPositions;

class ObjectListener {

private:

    std::string mapFrameId_;
    std::string objFramePrefix_;
    tf::TransformListener tfListener_;
    ros::Subscriber subs_;
    ros::Publisher pub_;
    std::vector<cleaner::ObjectPositions> positions_;

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
                        object_recognition::ObjectPosition position;
                        geometry_msgs::Pose coordinates;
                        // get data

                        //get object_id
                        int id = (int)msg->objects.data[i];
                        std::stringstream ss;
                        ss << "object_" << id;
                        std::string objectFrameId = ss.str();
                        position.object_id = id;
                        ROS_INFO("object_%d", id);

                        //get saved positions for current object
                        cleaner::ObjectPositions currentObject = getObjectPositions(objectFrameId);

                        //tf transformation
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

                        coordinates = assignCoordinates(pose);
                        position.pose = coordinates;
                        currentObject.positions.push_back(position);

                        int threshold = 5;

                        if (currentObject.positions.size() >= threshold) {
                            object_recognition::ObjectPosition aggregatedPosition = averageAggregation(currentObject);
                            publishPosition(aggregatedPosition);
                            deleteObjectPositions(currentObject);
                        }
                        writeBackObjectPositions(currentObject);

                    }
        }

    }

    void publishPosition(object_recognition::ObjectPosition position) {
        pub_.publish(position);
    }

    cleaner::ObjectPositions getObjectPositions(std::string objectId){
        for(int i = 0; i < positions_.size(); i++){
            if(positions_.at(i).objectId == objectId){
                return positions_.at(i);
            }
        }
        cleaner::ObjectPositions newObject;
        newObject.objectId = objectId;
        positions_.push_back(newObject);
        return newObject;
    }

    void writeBackObjectPositions(cleaner::ObjectPositions currentObject){
        for(int i = 0; i < positions_.size(); i++){
            if(positions_.at(i).objectId == currentObject.objectId){
                positions_.at(i) = currentObject;
                return;
            }
        }
    }

    void deleteObjectPositions(cleaner::ObjectPositions &currentObject){
        currentObject.positions.clear();
    }

    geometry_msgs::Pose assignCoordinates(tf::StampedTransform pose){
        geometry_msgs::Pose coordinates;
        coordinates.position.x = pose.getOrigin().x();
        coordinates.position.y = pose.getOrigin().y();
        coordinates.position.z = pose.getOrigin().z();
        coordinates.orientation.x = pose.getRotation().x();
        coordinates.orientation.y = pose.getRotation().y();
        coordinates.orientation.z = pose.getRotation().z();
        coordinates.orientation.w = pose.getRotation().w();
        return coordinates;
    }

    geometry_msgs::Pose addBiasedCoordinates(geometry_msgs::Pose finalCoordinates, geometry_msgs::Pose addedCoordinates, int div){
        finalCoordinates.position.x += addedCoordinates.position.x / div;
        finalCoordinates.position.y += addedCoordinates.position.y / div;
        finalCoordinates.position.z += addedCoordinates.position.z / div;
        finalCoordinates.orientation.x += addedCoordinates.orientation.x / div;
        finalCoordinates.orientation.y += addedCoordinates.orientation.y / div;
        finalCoordinates.orientation.z += addedCoordinates.orientation.z / div;
        finalCoordinates.orientation.w += addedCoordinates.orientation.w / div;
        return finalCoordinates;
    }

    void changeGraspingDegree(geometry_msgs::Quaternion &msgQuat){
        tf::Quaternion tfQuat;
        tf::quaternionMsgToTF(msgQuat,tfQuat);
        double roll, pitch, yaw;
        tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
        //ROS_INFO("Roll: %f",roll);
        roll += 0.8;
        tfQuat.setRPY(roll, pitch, yaw);
        /*tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
        ROS_INFO("Roll: %f",roll);*/
        tf::quaternionTFToMsg(tfQuat, msgQuat);
    }

    object_recognition::ObjectPosition averageAggregation(cleaner::ObjectPositions currentObject) {
        object_recognition::ObjectPosition aggregatedPosition;
        aggregatedPosition.object_id = currentObject.objectId;
        for (int i = 0; i < currentObject.positions.size(); i++){
            aggregatedPosition.pose = addBiasedCoordinates(aggregatedPosition.pose, currentObject.positions.at(i).pose, currentObject.positions.size());
        }
        if(true){ //think of a useful condition here
            changeGraspingDegree(aggregatedPosition.pose.orientation);
        }
        return aggregatedPosition;
    }

};

}

#endif

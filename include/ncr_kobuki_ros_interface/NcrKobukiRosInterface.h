//
// Created by jakub on 10. 4. 2020.
//

#ifndef SRC_NCRKOBUKIROSINTERFACE_H
#define SRC_NCRKOBUKIROSINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <ncr_kobuki_ros_interface/lidar_interface.h>
#include <ncr_kobuki_ros_interface/robot_interface.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class NcrKobukiRosInterface {
public:
    NcrKobukiRosInterface();

private:
    ros::Publisher laserScanPub;
    ros::Publisher odomDataPub;
    std::string tfPrefix;

    tf::TransformBroadcaster odomTfBroadcaster;

    sensor_msgs::LaserScan laserMeasurement2Msg(LaserMeasurement& measurement);

    geometry_msgs::Pose odomData2Msg(const RobotPose& robotPose);

    void sendOdomData(const geometry_msgs::Pose& odomPose);

    template <typename T>
    bool isValueInRange(T min, T max, T val);



};


#endif //SRC_NCRKOBUKIROSINTERFACE_H

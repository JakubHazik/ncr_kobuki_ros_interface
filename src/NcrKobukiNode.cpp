//
// Created by jakub on 10. 4. 2020.
//

#include <ros/ros.h>
#include <ncr_kobuki_ros_interface/lidar_interface.h>
#include <ncr_kobuki_ros_interface/robot_interface.h>


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "planner" );
    ros::NodeHandle nh("~");

    std::string IpAddress = "127.0.0.1";
    LidarInterface lidar(IpAddress);
    RobotInterface robot(IpAddress);

    ros::spin();
    return 0;
}

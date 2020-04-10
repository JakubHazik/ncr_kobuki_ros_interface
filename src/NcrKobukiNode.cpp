//
// Created by jakub on 10. 4. 2020.
//

#include <ros/ros.h>
#include <ncr_kobuki_ros_interface/NcrKobukiRosInterface.h>


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "planner" );

    NcrKobukiRosInterface rosInterface;

    return 0;
}

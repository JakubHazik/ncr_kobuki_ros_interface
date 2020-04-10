//
// Created by martin on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_LIDAR_INTERFACE_H
#define KOBUKI_PROJECT_LIDAR_INTERFACE_H

#include <mutex>
#include <thread>
#include <syslog.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <numeric>
#include <future>
#include <algorithm>

#include <ncr_kobuki_ros_interface/lidar.h>
#include <ncr_kobuki_ros_interface/own_typedefs.h>

using namespace std;

class LidarInterface {
public:
    explicit LidarInterface(const std::string& ipAddress);

    virtual ~LidarInterface();

    /**
     * Getter for laser data
     * @return <LaserMeasurement> structure
     */
    LaserMeasurement getLaserData();

    // Public for GUI - drawing
    std::mutex laserData_mtx;

private:
    std::string ipAddress;

    std::thread laser_thread;
    std::atomic_bool laserDataThreadRun = {true};
    LaserMeasurement laserData;

    Lidar lidar;

    /// UDP settings
    struct sockaddr_in socket_me, socket_other, socket_send;
    int socket_FD;
    ssize_t received_length;
    unsigned int socket_FD_length;

    char buf[50000];

    void t_readLaserData();

};

#endif //KOBUKI_PROJECT_LIDAR_INTERFACE_H

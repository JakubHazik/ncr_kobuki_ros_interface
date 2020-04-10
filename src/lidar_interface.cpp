//
// Created by martin on 14.3.2019.
//

#include <ncr_kobuki_ros_interface/lidar_interface.h>
#include <ros/ros.h>
#include <ncr_kobuki_ros_interface/file_lidar_sim.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rvt = rviz_visual_tools;

LidarInterface::LidarInterface(const std::string& ipAddress) {
    this->ipAddress = ipAddress;
    laser_thread = thread(&LidarInterface::t_readLaserData, this);
}

LidarInterface::~LidarInterface() {
    laserDataThreadRun = false;
    laser_thread.join();
}

void LidarInterface::t_readLaserData() {
    syslog(LOG_INFO, "[LidarInterface]: ReadLaserData thread started");
    // Initialize Winsock

    socket_FD_length = sizeof(socket_other);
    if ((socket_FD = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        syslog(LOG_ERR, "[LidarInterface]: Create socket failed");
    }

    int las_broadcastene = 1;
    setsockopt(socket_FD, SOL_SOCKET, SO_BROADCAST, (char *) &las_broadcastene, sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &socket_me, 0, sizeof(socket_me));

    socket_me.sin_family = AF_INET;
    socket_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    socket_me.sin_addr.s_addr = htonl(INADDR_ANY);//moze dojst od hocikial..

    socket_send.sin_family = AF_INET;
    socket_send.sin_port = htons(5299);//toto je port na ktory posielame
    socket_send.sin_addr.s_addr = inet_addr(ipAddress.c_str());//htonl(INADDR_BROADCAST);
    bind(socket_FD, (struct sockaddr *) &socket_me, sizeof(socket_me));

    syslog(LOG_INFO, "[LidarInterface]: Lidar sockets created, send empty command to lidar");

    //najskor posleme prazdny prikaz
    char command = 0x00;
    if (sendto(socket_FD, &command, sizeof(command), 0, (struct sockaddr *) &socket_send, socket_FD_length) == -1)//podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
    {
        syslog(LOG_ERR, "[LidarInterface]: Send empty command failed");
    }

    // for file scan data debug
//    rplidar lidar;
//    lidar.connectToFile("/home/jakub/amr_ws/src/ncr_kobuki_ros_interface/data/lidardata.txt");
//    rvt::RvizVisualToolsPtr visual_tools;
//    visual_tools.reset(new rvt::RvizVisualTools("base_scan", "/scan_marker"));
//    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting
//    visual_tools->deleteAllMarkers();
//    visual_tools->enableBatchPublishing();

    while (laserDataThreadRun) {
        // for file scan data debug
//        laserData = lidar.getMeasurementFromFile();
//        visual_tools->deleteAllMarkers();
//        for (int i = 0; i < laserData.numberOfScans; i++) {
//            geometry_msgs::Point point;
//            point.x = laserData.Data[i].scanDistance * cos(laserData.Data[i].scanAngle * DEG2RAD) /1000;
//            point.y = laserData.Data[i].scanDistance * sin(laserData.Data[i].scanAngle * DEG2RAD) / 1000;
//            visual_tools->publishSphere(point, rvt::colors::RED, rvt::scales::LARGE);
//        }
//        visual_tools->trigger();


        auto start = chrono::system_clock::now();
        if ((received_length = recvfrom(socket_FD, (char *) &laserData.Data, sizeof(LaserData) * 1000, 0, (struct sockaddr *) &socket_other, &socket_FD_length)) == -1) {
            continue;
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        long int duration = elapsed.count();

        if (duration > 200) {
            syslog(LOG_WARNING, "[LidarInterface]: Laser data not arrived %ld ms", duration);
        }

        start = chrono::system_clock::now();

        laserData.numberOfScans = received_length / sizeof(LaserData);

        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        duration = elapsed.count();

        if (duration > 50) {
            syslog(LOG_WARNING, "[LidarInterface]: Update local map takes so long time: %ld ms", duration);
        }
    }
}

LaserMeasurement LidarInterface::getLaserData() {
    lock_guard<mutex> lockGuard(laserData_mtx);
    return laserData;
}
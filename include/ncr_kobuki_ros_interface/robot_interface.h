//
// Created by jakub on 13.2.2019.
//

#ifndef KOBUKI_PROJECT_ROBOT_INTERFACE_H
#define KOBUKI_PROJECT_ROBOT_INTERFACE_H

#include <vector>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <future>

#include <ncr_kobuki_ros_interface/own_typedefs.h>

namespace HWConfig {
    // hardware params
    static const double TICK_TO_METER = 0.085292090497737556558;
    static const double TICK_TO_RAD = 0.002436916871363930187454;
    static const unsigned short ENCODER_MAX = 0xFFFF;                           // max of unsigned short
    static const int WHEEL_RADIUS = 35;                                         // [mm]
    static const int WHEEL_BASE = 230;                                          // [mm]
    static const int ROBOT_WIDTH = 360;                                         // mm
}

class RobotInterface {
public:
    explicit RobotInterface(std::string ipAddress);

    ~RobotInterface();

    void resetOdom(double x = 0, double y = 0, double fi = 0);

    RobotPose getOdomData();


    /*
     * Communication interface
     */

    void sendTranslationSpeed(int mmPerSec);

    void sendRotationSpeed(int radPerSec);

    void sendArcSpeed(int mmPerSec, int mmRadius);

private:
    /*
    * ========================================
    * Premenne
    */
    std::string ipAddress;

    std::thread robotDataRecv;
    std::atomic_bool robotDataThreadRun = {true};

    TKobukiData robotData;

    RobotPose odom;
    std::mutex odom_mtx;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other, rob_si_posli;
    int rob_s, rob_recv_len;
    unsigned int rob_slen;

    /*
     * ========================================
     * Funkcie
     */
    void t_readRobotData();

    void computeOdometry(unsigned short encoderRight, unsigned short encoderLeft, signed short gyroAngle);

    bool sendDataToRobot(const std::vector<unsigned char> &mess);


    std::vector<unsigned char> setTranslationSpeed(int mmpersec);

    std::vector<unsigned char> setRotationSpeed(double radpersec);

    std::vector<unsigned char> setArcSpeed(int mmpersec, int radius);

    std::vector<unsigned char> setSound(int noteinHz, int duration);

    std::vector<unsigned char> setDefaultPID();

    int checkChecksum(unsigned char *data);

    int parseKobukiMessage(TKobukiData &output, unsigned char *data);

    int fillData(TKobukiData &output, unsigned char *message) {
        return parseKobukiMessage(output, message);
    }
};

#endif //KOBUKI_PROJECT_ROBOT_INTERFACE_H

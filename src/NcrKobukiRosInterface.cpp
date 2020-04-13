//
// Created by jakub on 10. 4. 2020.
//

#include <ncr_kobuki_ros_interface/NcrKobukiRosInterface.h>
#include <tf/LinearMath/Quaternion.h>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <limits>

NcrKobukiRosInterface::NcrKobukiRosInterface() {
    std::string IpAddress = "127.0.0.1";
    lidarInterface = std::make_shared<LidarInterface>(IpAddress);
    robotInterface = std::make_shared<RobotInterface>(IpAddress);

    ros::NodeHandle nh("/");
    laserScanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 1, true);
    odomDataPub = nh.advertise<geometry_msgs::Pose>("odom", 1, true);
    cmdVelSub = nh.subscribe("mobile_base/commands/velocity", 5, &NcrKobukiRosInterface::cmdVelCb, this);

    ros::NodeHandle nhParam("~");
    nhParam.getParam("tf_prefix", tfPrefix);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();

        LaserMeasurement measurement = lidarInterface->getLaserData();
        if (measurement.numberOfScans > 0) {
            laserScanPub.publish(laserMeasurement2Msg(measurement));
        }

        auto robotOdomData = robotInterface->getOdomData();
        auto odomPose = odomData2Msg(robotOdomData);
        sendOdomData(odomPose);

        r.sleep();
    }
}

LaserData getClosestDataAngle(LaserMeasurement &measurement, float val) {
    int low = 0;
    int high = measurement.numberOfScans - 1;

    if (high < 0) {
        throw std::runtime_error("The array cannot be empty");
    }

    while (low < high) {
        int mid = (low + high) / 2;
        assert(mid < high);
        float d1 = std::abs(measurement.Data[mid  ].scanAngle - val);
        float d2 = std::abs(measurement.Data[mid + 1].scanAngle - val);
        if (d2 <= d1) {
            low = mid+1;
        } else {
            high = mid;
        }
    }

    return measurement.Data[high];
}


sensor_msgs::LaserScan NcrKobukiRosInterface::laserMeasurement2Msg(LaserMeasurement &measurement) {
    sensor_msgs::LaserScan scanMsg;
    scanMsg.header.frame_id = tf::resolve(tfPrefix, "base_scan");
    scanMsg.header.stamp = ros::Time::now();
    scanMsg.angle_min = 0;
    scanMsg.angle_max = M_2_PI;
    scanMsg.range_min = 0.15;
    scanMsg.range_max = 5;
    scanMsg.angle_increment = 2 * M_PI / measurement.numberOfScans;

    std::sort(measurement.Data, measurement.Data + measurement.numberOfScans, [](LaserData a, LaserData b) {
        return a.scanAngle < b.scanAngle;
    });

    float angle;
    for (int i = 0; i < measurement.numberOfScans; i++) {
        angle = i * scanMsg.angle_increment * RAD2DEG;
        auto laserData = getClosestDataAngle(measurement, angle);
        if (std::abs(angle - laserData.scanAngle) > 2) {
//            printf("error %f angle %f\n", std::abs(angle - laserData.scanAngle), angle);
            scanMsg.ranges.push_back(std::numeric_limits<float>::infinity());
            scanMsg.intensities.push_back(0);
            continue;
        }

        float distance = laserData.scanDistance / 1000;
        if (isValueInRange(scanMsg.range_min, scanMsg.range_max, distance)) {
            scanMsg.ranges.push_back(distance);
            scanMsg.intensities.push_back(laserData.scanQuality);
        } else {
            scanMsg.ranges.push_back(std::numeric_limits<float>::infinity());
            scanMsg.intensities.push_back(0);
        }
    }

    return scanMsg;
}

geometry_msgs::Pose NcrKobukiRosInterface::odomData2Msg(const RobotPose &robotPose) {
    geometry_msgs::Pose pose;
    pose.position.x = robotPose.x;
    pose.position.y = robotPose.y;

    tf::Quaternion q;
    q.setRPY(0, 0, robotPose.fi);

    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    return pose;
}

template<typename T>
bool NcrKobukiRosInterface::isValueInRange(T min, T max, T val) {
    return min < val && val < max;
}

void NcrKobukiRosInterface::sendOdomData(const geometry_msgs::Pose &odomPose) {
    // topic
    odomDataPub.publish(odomPose);

    // tf
    tf::Transform transform;
    tf::Pose tfPose;
    tf::poseMsgToTF(odomPose, tfPose);
    transform.setOrigin(tfPose.getOrigin());
    transform.setRotation(tfPose.getRotation());
    odomTfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
            tf::resolve(tfPrefix, "odom"),
            tf::resolve(tfPrefix, "base_footprint")));
}

int speedRadiusCorrection(int requiredSpeed, int radius) {

    float result = round(requiredSpeed + requiredSpeed * 0.05 - requiredSpeed/((fabs(radius) + 500.0)/500.0));

    if (result == 0) {
        result = 1;
    }

    // todo znamienka + saturacia
    return int(result);
}

void NcrKobukiRosInterface::cmdVelCb(const geometry_msgs::TwistConstPtr& msg) {

    int radius = static_cast<int>(msg->linear.x * 1000 / msg->angular.z / 2.0);
    int speed = static_cast<int>(msg->linear.x * 1000) * 2;
    robotInterface->sendArcSpeed(speed, radius);
}

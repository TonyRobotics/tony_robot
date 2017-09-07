#ifndef TRD_DIFF_CONTROLLER_H
#define TRD_DIFF_CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "trd_diff_controller/message_manager.h"

class TRDDiffController{
public:
    TRDDiffController();
    TRDDiffController(const char* serial_port_name, int baudrate);
    void cmdVelCallback(const geometry_msgs::Twist &msg);

private:
    ros::NodeHandle nh;
    ros::Publisher pub_imu;
    ros::Publisher pub_odom;
    ros::Subscriber sub_speed;
    sensor_msgs::Imu imu_msg;
    MessageManager message_manager;
    std::string serialport_name;
    int baudrate;
    double linear_coef;
    double angular_coef;
};

#endif


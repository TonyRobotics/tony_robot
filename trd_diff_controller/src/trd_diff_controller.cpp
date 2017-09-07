#include "trd_diff_controller/trd_diff_controller.h"

TRDDiffController::TRDDiffController(){
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serialport_name", serialport_name, "/dev/ttyUSB0");
    nh_private.param("baudrate", baudrate, 38400);
    nh_private.param("linear_coef", linear_coef, 320.0);
    nh_private.param("angular_coef", angular_coef, 60.0);
    ROS_INFO("serialport: %s", serialport_name.c_str());
    ROS_INFO("baudrate: %d", baudrate);
    if(message_manager.connect(serialport_name.c_str(), baudrate) < 0){
        return;
    }
    message_manager.setTimeout();
    message_manager.resetEncoder();
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 10);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
    sub_speed = nh.subscribe("/cmd_vel", 10, &TRDDiffController::cmdVelCallback, this);
    // ros spin
    ros::Rate loop_rate(10);
    while(nh.ok()){
        if(message_manager.getEncoderIMU() < 0){
            ROS_WARN("Get encoder_imu failed.");
        }
        else{
            imu_msg.linear_acceleration.x = message_manager.imu_linear_accel_x;
            imu_msg.linear_acceleration.y = message_manager.imu_linear_accel_y;
            imu_msg.linear_acceleration.z = message_manager.imu_linear_accel_z;
            imu_msg.angular_velocity.x = message_manager.imu_angular_vel_x;
            imu_msg.angular_velocity.y = message_manager.imu_angular_vel_y;
            imu_msg.angular_velocity.z = message_manager.imu_angular_vel_z;
            pub_imu.publish(imu_msg);
            //ROS_INFO("IMU angluar speed x: %f, y: %f, z: %f", \
            //        message_manager.imu_angular_vel_x, message_manager.imu_angular_vel_y, message_manager.imu_angular_vel_z);
            //ROS_INFO("IMU linear accel x: %f, y: %f, z: %f", \
            //        message_manager.imu_linear_accel_x, message_manager.imu_linear_accel_y, message_manager.imu_linear_accel_z);
            //ROS_INFO("IMU orientation x: %f, y: %f, z: %f", \
            //        message_manager.imu_orientation_x, message_manager.imu_orientation_y, message_manager.imu_orientation_z);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void TRDDiffController::cmdVelCallback(const geometry_msgs::Twist &msg){
    // Drive For- or Backward:
    int speed_l = round(linear_coef * msg.linear.x);
    int speed_r = round(linear_coef * msg.linear.x);
    // Turn clock- or counterclockwise:
    speed_l -= round(angular_coef * msg.angular.z);
    speed_r += round(angular_coef * msg.angular.z);        
    speed_l	+= 128;
    speed_r += 128;
    if(speed_l>255) speed_l = 255;
    if(speed_l<0)   speed_l = 0;
    if(speed_r>255) speed_r = 255;
    if(speed_r<0)   speed_r = 0; 
    message_manager.setSpeed(speed_l, speed_r);
    ROS_INFO("Set speed left: %x, right: %x", speed_l, speed_r);
}


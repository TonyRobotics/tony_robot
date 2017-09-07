#include "trd_diff_controller/message_manager.h"
#include "trd_diff_controller/serial_port.h"

#include "ros/ros.h"

MessageManager::MessageManager(){
    connect_flag = false;
}
int MessageManager::connect(const char *serial_port_name, const int baudrate){
    if(connect_flag){
        return -1;
    }
    if(openSerialPort(&serial_handler, serial_port_name) < 0){
        return -2;
    }
    if(setupSerialPort(serial_handler) < 0){
        return -3;
    }
    connect_flag = true;
    return 0;
}

int MessageManager::disconnect(){
    closeSerialPort(serial_handler);
    return 0;
}

int MessageManager::getEncoderIMU(){
    writeData(serial_handler, msg_get_encoder_imu.data, msg_get_encoder_imu.len);
    usleep(50000);
    rx_message.len = readData(serial_handler, rx_message.data, rx_message.EncoderIMU);
    if(rx_message.len == rx_message.EncoderIMU && rx_message.isMsgValid()){
        encoder_left   = (rx_message.data[3]&0xFF) << 24;
        encoder_left  |= (rx_message.data[4]&0xFF) << 16;
        encoder_left  |= (rx_message.data[5]&0xFF) << 8;
        encoder_left  |= (rx_message.data[6]&0xFF);
        encoder_right  = (rx_message.data[7]&0xFF) << 24;
        encoder_right |= (rx_message.data[8]&0xFF) << 16;
        encoder_right |= (rx_message.data[9]&0xFF) << 8;
        encoder_right |= (rx_message.data[10]&0xFF);
        ROS_INFO("Encoder left: %d, right: %d", encoder_left, encoder_right);
        int16_t imu_tmp;
        imu_tmp  = (rx_message.data[11]&0xFF) << 8;
        imu_tmp |= (rx_message.data[12]&0xFF);
        imu_angular_vel_x = 2000.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[13]&0xFF) << 8;
        imu_tmp |= (rx_message.data[14]&0xFF);
        imu_angular_vel_y = 2000.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[15]&0xFF) << 8;
        imu_tmp |= (rx_message.data[16]&0xFF);
        imu_angular_vel_z = 2000.0*imu_tmp / 32768;

        imu_tmp  = (rx_message.data[17]&0xFF) << 8;
        imu_tmp |= (rx_message.data[18]&0xFF);
        imu_linear_accel_x = 16.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[19]&0xFF) << 8;
        imu_tmp |= (rx_message.data[20]&0xFF);
        imu_linear_accel_y = 16.0*imu_tmp / 32768;
        imu_tmp  = (rx_message.data[21]&0xFF) << 8;
        imu_tmp |= (rx_message.data[22]&0xFF);
        imu_linear_accel_z = 16.0*imu_tmp / 32768;

        imu_tmp  = (rx_message.data[23]&0xFF) << 8;
        imu_tmp |= (rx_message.data[24]&0xFF);
        imu_orientation_x = 1.0*imu_tmp / 100;
        imu_tmp  = (rx_message.data[25]&0xFF) << 8;
        imu_tmp |= (rx_message.data[26]&0xFF);
        imu_orientation_y = 1.0*imu_tmp / 100;
        imu_tmp  = (rx_message.data[27]&0xFF) << 8;
        imu_tmp |= (rx_message.data[28]&0xFF);
        imu_orientation_z = 1.0*imu_tmp / 100;
        return 0;
    }
    return -1;
}
void MessageManager::setSpeed(char speed_left, char speed_right){
    msg_set_speed_left.loadSpeed(speed_left);
    msg_set_speed_right.loadSpeed(speed_right);
    writeData(serial_handler, msg_set_speed_left.data, msg_set_speed_left.len);
    usleep(1000);
    writeData(serial_handler, msg_set_speed_right.data, msg_set_speed_right.len);
    ROS_INFO("Set Speed OK");
}
void MessageManager::setMode(){
}

void MessageManager::setTimeout(){
    writeData(serial_handler, msg_set_timeout.data, msg_set_timeout.len);
    ROS_INFO("Set timeout OK");
}
void MessageManager::resetEncoder(){
    writeData(serial_handler, msg_reset_encoder.data, msg_reset_encoder.len);
    ROS_INFO("Reset encoder OK");
}
void MessageManager::resetBase(){
    writeData(serial_handler, msg_reset_base.data, msg_reset_base.len);
    ROS_INFO("Reset base OK");
}


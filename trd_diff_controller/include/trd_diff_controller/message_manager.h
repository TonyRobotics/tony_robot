#ifndef MESSAGE_MANAGER_H
#define MESSAGE_MANAGER_H

#include "stdint.h"

char crc8(int size, char init_val, char *data);

#define SEND_MAXLEN 32
class SendMessage{
public:
    int len;
    char data[SEND_MAXLEN];
    void makeMsgValid();
};
class SendMessageGetEncoder : public SendMessage{
public:
    SendMessageGetEncoder();
};
class SendMessageGetIMU : public SendMessage{
public:
    SendMessageGetIMU();
};
class SendMessageGetEncoderIMU : public SendMessage{
public:
    SendMessageGetEncoderIMU();
};
class SendMessageSetSpeedLeft : public SendMessage{
public:
    SendMessageSetSpeedLeft();
    void loadSpeed(char c);
};
class SendMessageSetSpeedRight : public SendMessage{
public:
    SendMessageSetSpeedRight();
    void loadSpeed(char c);
};
class SendMessageSetTimeout : public SendMessage{
public:
    SendMessageSetTimeout();
};
class SendMessageResetEncoder : public SendMessage{
public:
    SendMessageResetEncoder();
};
class SendMessageResetBase : public SendMessage{
public:
    SendMessageResetBase();
};

#define RX_MAXLEN 256
class RxMessage{
public:
    RxMessage();
    RxMessage(const char *reply, const int n);
    int len;
    char data[RX_MAXLEN];
    bool isMsgValid();
    enum MsgType{
        Mode = 6,
        ERROR = 6,
        VC = 8,
        Encoder = 13,
        IMU = 23,
        EncoderIMU = 31
    };
};

class MessageManager{
public:
    MessageManager();
    int connect(const char *serial_port_name, const int baudrate);
    int disconnect();
    // get
    void getEncoderIMU();
    // set
    void setSpeed(char speed_left, char speed_right);
    void setMode();
    void setTimeout();
    void resetEncoder();
    void resetBase();
    int32_t encoder_left, encoder_right;
    double imu_linear_accel_x, imu_linear_accel_y, imu_linear_accel_z; 
    double imu_angular_vel_x, imu_angular_vel_y, imu_angular_vel_z;
    double imu_orientation_x, imu_orientation_y, imu_orientation_z;
private:
    void *serial_handler;
    bool connect_flag;
    // send get msg
    SendMessageGetEncoder msg_get_encoder;
    SendMessageGetIMU msg_get_imu;
    SendMessageGetEncoderIMU msg_get_encoder_imu;
    // send set msg
    SendMessageSetSpeedLeft msg_set_speed_left;
    SendMessageSetSpeedRight msg_set_speed_right;
    SendMessageSetTimeout msg_set_timeout;
    SendMessageResetEncoder msg_reset_encoder;
    SendMessageResetBase msg_reset_base;
    // rx msg
    RxMessage rx_message;
};

#endif

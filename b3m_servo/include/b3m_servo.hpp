#ifndef B3M_SERVO_H_
#define B3M_SERVO_H_

#include "serial.hpp"

#define NORMAL_MODE 0x00
#define FREE_MODE 0x02
#define HOLD_MODE 0x03

#define TRAJECTORY_NORMAL_MODE 0
#define TRAJECTORY_EVEN_MODE 1
#define TRAJECTORY_THIRDPOLY_MODE 3
#define TRAJECTORY_FORTHPOLY_MODE 4
#define TRAJECTORY_FIFTHPOLY_MODE 5

class KondoB3mServo{
 public:
  KondoB3mServo(){}

  int b3mNormalPosModeSet(SerialPort *port, unsigned char servoID)
  {
    return b3mTorquModeSet(port, servoID, NORMAL_MODE);
  }
  
  int b3mFreePosModeSet(SerialPort *port, unsigned char servoID)
  {
    return b3mTorquModeSet(port, servoID, FREE_MODE);
  }

  int b3mTorquModeSet(SerialPort *port, unsigned char servoID, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];

    ChangeServoStatus(0x00, 1, servoID, mode, sendData);
    return write(port->fd_, sendData, sizeof(sendData));
  }

  void ChangeServoStatus(unsigned char option, unsigned char count, unsigned char servoID, unsigned char mode, unsigned char data[])
  {
    data[0]  = (unsigned char)8; // SIZE
    data[1]  = (unsigned char)0x04;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)servoID; //id
    data[4]  = (unsigned char)mode; // DATA
    data[5]  = (unsigned char)0x28; // ADDRESS
    data[6]  = (unsigned char)count; // 指定するデバイスの数 CNT
    data[7]  = checksum(data, 7);// チェックサム
  }

  int b3mTrajectoryModeSet(SerialPort *port, unsigned char servoID, unsigned char mode)
  {
    unsigned char sendData[8];
    ChangeTrajectoryMode(0x00, 1, servoID, mode, sendData);
    return write(port->fd_, sendData, sizeof(sendData));
  }

  void  ChangeTrajectoryMode(unsigned char option, unsigned char count, unsigned char servoID, unsigned char mode, unsigned char data[])
  {
    data[0]  = (unsigned char)8; // SIZE
    data[1]  = (unsigned char)0x04;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)servoID; //id
    data[4]  = (unsigned char)mode; // DATA
    data[5]  = (unsigned char)0x29; // ADDRESS
    data[6]  = (unsigned char)count; // 指定するデバイスの数 CNT
    data[7]  = checksum(data, 7);// チェックサム
  }

  int b3mGainParamSet(SerialPort *port, unsigned char servoID, unsigned char mode)
  {
    unsigned char sendData[8];
    ChangeServoGain(0x00, 1, servoID, mode, sendData);
    return write(port->fd_, sendData, sizeof(sendData));
  }

  void ChangeServoGain( unsigned char option, unsigned char count, unsigned char servoID, unsigned char mode, unsigned char data[])
  {
    data[0]  = (unsigned char)8; // SIZE
    data[1]  = (unsigned char)0x04;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)servoID; //id
    data[4]  = (unsigned char)mode; // DATA
    data[5]  = (unsigned char)0x5c; // ADDRESS
    data[6]  = (unsigned char)count; // 指定するデバイスの数 CNT
    data[7]  = checksum(data, 7);// チェックサム
  }
  
  int b3mSetPosition(SerialPort *port, unsigned char servoID, short angle, short target_time)
  {
    unsigned char sendData[9];
    SetServoPosition(0x00, servoID, angle, target_time, sendData);
    return write(port->fd_, sendData, sizeof(sendData));
  }

  void SetServoPosition(unsigned char option, unsigned char servoID, short angle, short target_time, unsigned char data[])
  {
    data[0]  = (unsigned char)9; // SIZE
    data[1]  = (unsigned char)0x06;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)servoID; //id
    data[4]  = (unsigned char)(angle&0x00FF); // POS_L
    data[5]  = (unsigned char)((angle&0xFF00)>>8); // POS_H
    data[6]  = (unsigned char)(target_time&0x00FF); // TIME_L
    data[7]  = (unsigned char)((target_time&0xFF00)>>8); // TIME_H
    data[8]  = checksum(data, 8);// チェックサム
  }

 private:  
  unsigned char checksum(unsigned char data[], int num)
  {
    short sum = 0;
    for (int i = 0; i < num; ++i) {
      sum += data[i];
    }
    return (unsigned char)(sum&0x00FF); // SIZE~TIMEまでの総和の下位1Byte
  }
};

#endif

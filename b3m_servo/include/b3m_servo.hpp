#ifndef B3M_SERVO_H_
#define B3M_SERVO_H_

#include <ros/ros.h>
#include <string>
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
  int min_angle_;
  int max_angle_;
  unsigned char id_;
  std::string joint_name_;

  KondoB3mServo(std::string actuator_name)
  {
    ros::NodeHandle nh(std::string("~")+actuator_name);
    int id_int=0;
    if(nh.getParam("id", id_int))
    {
      id_ = (unsigned char)id_int;
      ROS_INFO("id: %d", id_);
    }

	if (nh.getParam("joint_name", joint_name_)) {
      ROS_INFO("joint_name: %s", joint_name_.c_str());	    
	}

	if (nh.getParam("min_angle", min_angle_)) {
      ROS_INFO("min_angle: %d", min_angle_);
	}

	if (nh.getParam("max_angle", max_angle_)) {
      ROS_INFO("max_angle: %d", max_angle_);
	}
  }

  int b3mNormalPosModeSet(SerialPort *port)
  {
    return b3mTorquModeSet(port, NORMAL_MODE);
  }
  
  int b3mFreePosModeSet(SerialPort *port)
  {
    return b3mTorquModeSet(port, FREE_MODE);
  }

  int b3mTorquModeSet(SerialPort *port, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];

    ChangeServoStatus(0x00, 1, mode, sendData);
    printf("tom)send : ");
    for(int i = 0; i < sizeof(sendData); ++i)
	{
	  printf("%x  ", sendData[i]);
	}
    printf("\n");
    write(port->fd_, sendData, sizeof(sendData));
    read(port->fd_, receiveData, sizeof(receiveData));
    printf("tom)received : ");
    for(int i = 0; i < sizeof(receiveData); ++i)
	{
	  printf("%x  ", receiveData[i]);
	}
    printf("\n");
    printf("-------------------------------\n");
    return 0;
  }

  void ChangeServoStatus(unsigned char option, unsigned char count, unsigned char mode, unsigned char data[])
  {
    data[0]  = (unsigned char)8; // SIZE
    data[1]  = (unsigned char)0x04;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)id_; //id
    data[4]  = (unsigned char)mode; // DATA
    data[5]  = (unsigned char)0x28; // ADDRESS
    data[6]  = (unsigned char)count; // 指定するデバイスの数 CNT
    data[7]  = checksum(data, 7);// チェックサム
  }

  int b3mTrajectoryModeSet(SerialPort *port, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];
    ChangeTrajectoryMode(0x00, 1, mode, sendData);
    printf("trm)send : ");
    for(int i = 0; i < sizeof(sendData); ++i)
	{
	  printf("%x  ", sendData[i]);
	}
    printf("\n");
    write(port->fd_, sendData, sizeof(sendData));
    read(port->fd_, receiveData, sizeof(receiveData));
    printf("trm)received : ");
    for(int i = 0; i < sizeof(receiveData); ++i)
	{
	  printf("%x  ", receiveData[i]);
	}
    printf("\n");
    printf("-------------------------------\n");
    return 0;
  }

  void  ChangeTrajectoryMode(unsigned char option, unsigned char count, unsigned char mode, unsigned char data[])
  {
    data[0]  = (unsigned char)8; // SIZE
    data[1]  = (unsigned char)0x04;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)id_; //id
    data[4]  = (unsigned char)mode; // DATA
    data[5]  = (unsigned char)0x29; // ADDRESS
    data[6]  = (unsigned char)count; // 指定するデバイスの数 CNT
    data[7]  = checksum(data, 7);// チェックサム
  }

  int b3mGainParamSet(SerialPort *port, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];
    ChangeServoGain(0x00, 1, mode, sendData);
    write(port->fd_, sendData, sizeof(sendData));
    printf("gp)send : ");
    for(int i = 0; i < sizeof(sendData); ++i)
	{
	  printf("%x  ", sendData[i]);
	}
    printf("\n");
    read(port->fd_, receiveData, sizeof(receiveData));
    printf("gp)received : ");
    for(int i = 0; i < sizeof(receiveData); ++i)
	{
	  printf("%x  ", receiveData[i]);
	}
    printf("\n");
    printf("-------------------------------\n");
    return 0;
  }

  void ChangeServoGain( unsigned char option, unsigned char count, unsigned char mode, unsigned char data[])
  {
    data[0]  = (unsigned char)8; // SIZE
    data[1]  = (unsigned char)0x04;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)id_; //id
    data[4]  = (unsigned char)mode; // DATA
    data[5]  = (unsigned char)0x5c; // ADDRESS
    data[6]  = (unsigned char)count; // 指定するデバイスの数 CNT
    data[7]  = checksum(data, 7);// チェックサム
  }
  
  int b3mSetPosition(SerialPort *port, short angle, short target_time)
  {
    unsigned char sendData[9];
    unsigned char receiveData[7];
    if(angle > max_angle_*100)
    {
      angle = max_angle_*100;
    }
    if(angle < min_angle_*100)
    {
      angle = min_angle_*100;
    }
    SetServoPosition(0x00, angle, target_time, sendData);
    write(port->fd_, sendData, sizeof(sendData));

    printf("sp)send : ");
    for(int i = 0; i < sizeof(sendData); ++i)
	{
	  printf("%x  ", sendData[i]);
	}
    printf("\n");

    read(port->fd_, receiveData, sizeof(receiveData));

    printf("sp)received : ");
    for(int i = 0; i < sizeof(receiveData); ++i)
	{
	  printf("%x  ", receiveData[i]);
	}
    printf("\n");
    printf("-------------------------------\n");
    return 0;
  }

  void SetServoPosition(unsigned char option, short angle, short target_time, unsigned char data[])
  {
    data[0]  = (unsigned char)9; // SIZE
    data[1]  = (unsigned char)0x06;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    data[3]  = (unsigned char)id_; //id
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

class KondoB3mServoMultiCtrl
{
 private:
  std::vector<boost::shared_ptr<KondoB3mServo> > actuator_vector_;
  int size_of_data_;
  int num_of_servo_;
 public:
  KondoB3mServoMultiCtrl(std::vector<boost::shared_ptr<KondoB3mServo> > actuator_vector)
  {
    actuator_vector_ = actuator_vector;
    num_of_servo_ = actuator_vector_.size();
    size_of_data_ = actuator_vector_.size() * 3 + 3;
  }
  
  void b3mSetPositionMulti(SerialPort *port, std::vector<short> angles, short target_time)
  {
    unsigned char *sendData = new unsigned char[size_of_data_];
    SetServoPositionMulti(0x00, num_of_servo_, angles, target_time, sendData);
    write(port->fd_, sendData, sizeof(sendData));

    printf("sp)send : ");
    for(int i = 0; i < sizeof(sendData); ++i)
	{
	  printf("%x  ", sendData[i]);
	}
    printf("\n");
  }
  
  void SetServoPositionMulti(unsigned char option, int num, std::vector<short> angles, short target_time, unsigned char data[])
  {
    data[0]  = (unsigned char)size_of_data_; // SIZE
    data[1]  = (unsigned char)0x06;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    for (int i = 0; i < num_of_servo_; ++i) {
      data[3 + 3*i] = (unsigned char)actuator_vector_[i]->id_;
      data[4 + 3*i] = (unsigned char)(angles[i]&0x00FF); // POS_L
      data[5 + 3*i] = (unsigned char)((angles[i]&0xFF00)>>8); // POS_H
    }
    data[3 + 3*num_of_servo_]  = (unsigned char)(target_time&0x00FF); // TIME_L
    data[3 + 3*num_of_servo_ + 1]  = (unsigned char)((target_time&0xFF00)>>8); // TIME_H
    data[size_of_data_ -1]  = checksum(data, size_of_data_-1);// チェックサム
  }
  
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

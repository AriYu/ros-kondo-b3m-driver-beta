#ifndef B3M_SERVO_H_
#define B3M_SERVO_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
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

unsigned char checksum(unsigned char data[], int num);
int hexa2dec(unsigned char data1, unsigned char data2);

class KondoB3mServo{
 public:
  int min_angle_;
  int max_angle_;
  double angle_;
  unsigned char id_;
  std::string joint_name_;
  
  KondoB3mServo(std::string actuator_name)
  {
    ros::NodeHandle nh(std::string("~")+actuator_name);
    int id_int=0;
    if(nh.getParam("id", id_int)){
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

  void setId(int new_id)
  {
    id_ = new_id;
  }

  int setNormalPosMode(SerialPort *port)
  {
    return setTorquMode(port, NORMAL_MODE);
  }
  
  int setFreePosMode(SerialPort *port)
  {
    return setTorquMode(port, FREE_MODE);
  }

  int setTorquMode(SerialPort *port, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];

    generateChangeServoStatusCmd(0x00, 1, mode, sendData);
    write(port->fd_, sendData, sizeof(sendData));
    read(port->fd_, receiveData, sizeof(receiveData));
    return 0;
  }

  void generateChangeServoStatusCmd(unsigned char option, unsigned char count, unsigned char mode, unsigned char data[])
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

  int setTrajectoryMode(SerialPort *port, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];
    generateChangeTrajectoryModeCmd(0x00, 1, mode, sendData);
    write(port->fd_, sendData, sizeof(sendData));
    read(port->fd_, receiveData, sizeof(receiveData));
    return 0;
  }

  void generateChangeTrajectoryModeCmd(unsigned char option, unsigned char count, unsigned char mode, unsigned char data[])
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

  int setGainParam(SerialPort *port, unsigned char mode)
  {
    unsigned char sendData[8];
    unsigned char receiveData[5];
    generateChangeServoGainCmd(0x00, 1, mode, sendData);
    write(port->fd_, sendData, sizeof(sendData));
    read(port->fd_, receiveData, sizeof(receiveData));
    return 0;
  }

  void generateChangeServoGainCmd( unsigned char option, unsigned char count, unsigned char mode, unsigned char data[])
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
  
  int setPosition(SerialPort *port, short angle, short target_time)
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
    generateSetServoPositionCmd(0x00, angle, target_time, sendData);
    write(port->fd_, sendData, sizeof(sendData));
    read(port->fd_, receiveData, sizeof(receiveData));
    return 0;
  }

  void generateSetServoPositionCmd(unsigned char option, short angle, short target_time, unsigned char data[])
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

  int readPosition(SerialPort *port)
  {
    unsigned char sendData[7] = {};
    unsigned char receiveData[7] = {};
    int check = 0;
    generateReadServoPositionCmd(sendData);
    
    while(1){
      tcflush(port->fd_, TCIFLUSH);      // 受信バッファのフラッシュ
      tcflush(port->fd_, TCOFLUSH);      // 送信バッファのフラッシュ
      write(port->fd_, sendData, sizeof(sendData));
      read(port->fd_, receiveData, sizeof(receiveData));
      int check = checksum(receiveData, 6);
      if(check == receiveData[6])
      {
        break;
      }
      usleep(5000);
    }
    // Data[4]が下位2bit, Data[5]が上位2bit
    int angle = hexa2dec(receiveData[4], receiveData[5]);
    angle_ = (double)angle/100.0;
    return angle;
  }

  void generateReadServoPositionCmd(unsigned char data[])
  {
    data[0] = (unsigned char)0x07;
    data[1] = (unsigned char)0x03;
    data[2] = (unsigned char)0x00;
    data[3] = (unsigned char)id_;
    data[4] = (unsigned char)0x2C;
    data[5] = (unsigned char)0x02;
    data[6] = checksum(data, 6);
  }

  double getAngle()
  {
    return angle_;
  }
  
};

class KondoB3mServoMultiCtrl
{
 private:
  std::vector<boost::shared_ptr<KondoB3mServo> > actuator_vector_;
  int size_of_data_;
  int num_of_servo_;
  boost::mutex access_mutex_;
  
 public:
  KondoB3mServoMultiCtrl(std::vector<boost::shared_ptr<KondoB3mServo> > actuator_vector)
  {
    actuator_vector_ = actuator_vector;
    num_of_servo_ = actuator_vector_.size();
    size_of_data_ = actuator_vector_.size() * 3 + 6;
  }
  
  void setPositionMulti(SerialPort *port, std::vector<short> angles, short target_time)
  {
    unsigned char *sendData = new unsigned char[size_of_data_];
    setServoPositionMulti(0x00, num_of_servo_, angles, target_time, sendData);
    {
      boost::mutex::scoped_lock(access_mutex_);
      write(port->fd_, sendData, size_of_data_);
    }

  }
  
  void setServoPositionMulti(unsigned char option, int num, std::vector<short> angles, short target_time, unsigned char data[])
  {
    data[0]  = (unsigned char)size_of_data_; // SIZE
    data[1]  = (unsigned char)0x06;  // コマンド(write)
    data[2]  = (unsigned char)0x00; // OPTION
    for (int i = 0; i < num_of_servo_; ++i) {
      data[3 + 3*i] = (unsigned char)actuator_vector_[i]->id_;
      if(angles[i] > actuator_vector_[i]->max_angle_*100)
      {
        angles[i] = actuator_vector_[i]->max_angle_*100;
      }
      if(angles[i] < actuator_vector_[i]->min_angle_*100)
      {
        angles[i] = actuator_vector_[i]->min_angle_*100;
      }
      data[4 + 3*i] = (unsigned char)(angles[i]&0x00FF); // POS_L
      data[5 + 3*i] = (unsigned char)((angles[i]&0xFF00)>>8); // POS_H
    }
    data[3 + 3*num_of_servo_]  = (unsigned char)(target_time&0x00FF); // TIME_L
    data[3 + 3*num_of_servo_ + 1]  = (unsigned char)((target_time&0xFF00)>>8); // TIME_H
    data[size_of_data_ -1]  = checksum(data, size_of_data_-1);// チェックサム
  }

  void readPositionMulti(SerialPort *port)
  {
    {
      boost::mutex::scoped_lock(access_mutex_);
      for (size_t i = 0; i < actuator_vector_.size(); ++i) {
	actuator_vector_[i]->readPosition(port);
      }
    }
  }

};

unsigned char checksum(unsigned char data[], int num)
{
  short sum = 0;
  for (int i = 0; i < num; ++i) {
    sum += data[i];
  }
  return (unsigned char)(sum&0x00FF); // SIZE~TIMEまでの総和の下位1Byte
}

int hexa2dec(unsigned char data1, unsigned char data2)
{
  int return_val = 0;
  // 負の数(2二進数で最上位ビットが1)
  if(data2 > 127)
  {
    unsigned char minus_data1;
    unsigned char minus_data2;
    minus_data1 = ~data1; // ビット反転
    minus_data2 = ~data2; // ビット反転
    // +1する
    if(minus_data1 < 255){
      minus_data1 = minus_data1 + 1;
    }else{
      minus_data2 = minus_data2 + 1;
    }
    return_val = (minus_data1 % 16)*1 + (minus_data1 / 16)*16 
        + (minus_data2 % 16)*pow(16, 2) + (minus_data2 / 16)*pow(16, 3);
    return_val = -return_val;
  }else
  {
    return_val = (data1 % 16)*1 + (data1 / 16)*16 
        + (data2 % 16)*pow(16, 2) + (data2 / 16)*pow(16, 3);
  }
  return return_val;
}

#endif

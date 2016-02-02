#include <ros/ros.h>
#include <serial.hpp>
#include <b3m_servo.hpp>
#include <string>

// 今の角度を中心位置に設定するプログラム
// 1. 指定したIDの角度を読み取る
// 2. 読み取った角度を中心オフセットにして書き込む
// 3. ROMに保存する

class b3m_set_center
{
 public:
  b3m_set_center(std::string portName, int baudrate, int id)
      : port_(portName, baudrate), servo_("servo")
  {
    servo_.b3mSetId(id);
    double angle = servo_.b3mReadPosition(&port_);
    ROS_INFO("Angle : %lf", angle);
  }

  ~b3m_set_center()
  {
  }

 private:
  KondoB3mServo servo_;
  SerialPort port_;
};


int main(int argc, char **argv)
{
  
  b3m_set_center b3m_setter("/dev/ttyUSB0", B115200, atoi(argv[1]));

  return 0;
}

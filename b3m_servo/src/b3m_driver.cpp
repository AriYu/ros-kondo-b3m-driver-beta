#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial.hpp>
#include <b3m_servo.hpp>
#include <string>

#include <boost/thread.hpp>

class b3m_servo_driver
{
 public:
  b3m_servo_driver(ros::NodeHandle nh, std::string portName, int baudrate)
      : nh_(nh), rate_(20), port_(portName, baudrate), servo_id_(0)
  {

    servo_.b3mFreePosModeSet(&port_, servo_id_);
    servo_.b3mNormalPosModeSet(&port_, servo_id_);
    servo_.b3mTrajectoryModeSet(&port_, servo_id_, TRAJECTORY_EVEN_MODE);
    servo_.b3mGainParamSet(&port_, servo_id_, 0x00);
    joint_angle_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(&b3m_servo_driver::joint_cb, this, _1));
  }
  void joint_cb(const sensor_msgs::JointStateConstPtr& joint_state)
  {
    short angle = (short)(joint_state->position[0] * 100);
    short target_time = 0;
    ROS_INFO("Angle : %d\n",angle);
    servo_.b3mSetPosition(&port_, servo_id_, angle, target_time);
    usleep(1000);
  }
  void run()
  {
    while(nh_.ok())
	{
	  ros::spinOnce();
	  rate_.sleep();
    }
  }
  ~b3m_servo_driver()
  {
    servo_.b3mFreePosModeSet(&port_, servo_id_);
  }
 private:
  KondoB3mServo servo_;
  SerialPort port_;
  int servo_id_;
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber joint_angle_sub_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "b3m_servo_node");
  ros::NodeHandle nh;
  
  b3m_servo_driver  driver(nh, "/dev/ttyUSB0", B115200);
  driver.run();


  return 0;
}

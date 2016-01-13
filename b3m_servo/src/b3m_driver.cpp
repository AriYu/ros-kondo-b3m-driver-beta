#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial.hpp>
#include <b3m_servo.hpp>
#include <string>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

class b3m_servo_driver
{
 public:
  b3m_servo_driver(ros::NodeHandle nh, std::string portName, int baudrate, int num, char** actuators_name)
      : nh_(nh), rate_(20), port_(portName, baudrate), loop_(0)
  {
    for (int i = 0; i < num; i++) {
      boost::shared_ptr<KondoB3mServo> actuator(new KondoB3mServo(std::string(actuators_name[i])));
      actuator_vector_.push_back(actuator);
	}
    for (int i = 0; i < num; i++) {
      actuator_vector_[i]->b3mFreePosModeSet(&port_);
      actuator_vector_[i]->b3mNormalPosModeSet(&port_);
      actuator_vector_[i]->b3mTrajectoryModeSet(&port_, TRAJECTORY_EVEN_MODE);
      actuator_vector_[i]->b3mGainParamSet(&port_, 0x00);
    }

    angles_.resize(actuator_vector_.size());
    multi_ctrl = new KondoB3mServoMultiCtrl(actuator_vector_);

    joint_angle_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(&b3m_servo_driver::joint_cb, this, _1));

  }

  void joint_cb(const sensor_msgs::JointStateConstPtr& joint_state)
  {
    short target_time = 0;
    ROS_INFO("loop : %d", loop_);
    loop_++;
    for(int i = 0; i < actuator_vector_.size(); ++i)
    {
      angles_[i] = (short)(joint_state->position[i] * 100 * 100);
    }
    multi_ctrl->b3mSetPositionMulti(&port_, angles_, target_time);
    // for (int i = 0; i < actuator_vector_.size(); ++i) {
    //   short angle = (short)(joint_state->position[i] * 100 * 100);
    //   ROS_INFO("id : %d, Angle : %d", i, angle);
    //   actuator_vector_[i]->b3mSetPosition(&port_, angle, target_time);
    //   usleep(10000);
    // }
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
    for (int i = 0; i < actuator_vector_.size(); ++i) {
      actuator_vector_[i]->b3mFreePosModeSet(&port_);
      usleep(5000);
    }
    delete multi_ctrl;
  }

 private:
  std::vector<boost::shared_ptr<KondoB3mServo> > actuator_vector_;
  KondoB3mServoMultiCtrl *multi_ctrl;
  SerialPort port_;
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber joint_angle_sub_;
  int loop_;
  std::vector<short> angles_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "b3m_driver");
  ros::NodeHandle nh;
  
  b3m_servo_driver  driver(nh, "/dev/ttyUSB0", B115200, argc-1, &argv[1]);
  driver.run();


  return 0;
}

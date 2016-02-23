#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial.hpp>
#include <b3m_servo.hpp>
#include <string>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

class B3mServoDriver
{
 public:
  B3mServoDriver(ros::NodeHandle nh, std::string portName, int baudrate, int num, char** actuators_name)
      : nh_(nh), rate_(20), port_(portName, baudrate), loop_(0)
  {
    for (int i = 0; i < num; i++) {
      boost::shared_ptr<KondoB3mServo> actuator(new KondoB3mServo(std::string(actuators_name[i])));
      actuator_vector_.push_back(actuator);
      joint_states_.name.push_back(std::string(actuators_name[i]));
    }
    
    joint_states_.position.resize(num);
    joint_states_.velocity.resize(num);
    joint_states_.effort.resize(num);
   
    for (int i = 0; i < num; i++) {
      actuator_vector_[i]->b3mFreePosModeSet(&port_);
      usleep(10000);
      actuator_vector_[i]->b3mNormalPosModeSet(&port_);
      usleep(10000);
      actuator_vector_[i]->b3mTrajectoryModeSet(&port_, TRAJECTORY_EVEN_MODE);
      usleep(10000);
      actuator_vector_[i]->b3mGainParamSet(&port_, 0x00);
      usleep(10000);
    }
    
    angles_.resize(actuator_vector_.size());
    multi_ctrl_ = new KondoB3mServoMultiCtrl(actuator_vector_);

    ros::NodeHandle n("~");
    joint_angle_sub_ = nh_.subscribe<sensor_msgs::JointState>(n.param<std::string>("joint_cmd_topic_name", "/joint_cmd"), 1, boost::bind(&B3mServoDriver::jointPositionCallback, this, _1));
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(n.param<std::string>("joint_state_topic_name", "/joint_states"), 10);
  }

  void jointPositionCallback(const sensor_msgs::JointStateConstPtr& joint_state)
  {
    short target_time = 0;
    double angle_deg = 0;
    for(int i = 0; i < actuator_vector_.size(); ++i)
    {
      angle_deg = (joint_state->position[i]*180.0)/M_PI;
      angles_[i] = (short)(angle_deg * 100);
    }
    multi_ctrl_->b3mSetPositionMulti(&port_, angles_, target_time);
    usleep(10000);
  }
  
  void run()
  {
    while(nh_.ok())
	{
	  ros::spinOnce();
	  rate_.sleep();
    }
  }

  ~B3mServoDriver()
  {
    for (int i = 0; i < actuator_vector_.size(); ++i) {
      actuator_vector_[i]->b3mFreePosModeSet(&port_);
      usleep(5000);
    }
    delete multi_ctrl_;
  }

 private:
  std::vector<boost::shared_ptr<KondoB3mServo> > actuator_vector_;
  KondoB3mServoMultiCtrl *multi_ctrl_;
  SerialPort port_;
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber joint_angle_sub_;
  int loop_;
  std::vector<short> angles_;

  ros::Publisher joint_state_pub_;
  sensor_msgs::JointState joint_states_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "b3m_driver");
  ros::NodeHandle nh;
  
  B3mServoDriver driver(nh, "/dev/ttyUSB0", B115200, argc-1, &argv[1]);
  driver.run();

  return 0;
}

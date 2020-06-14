#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <carolo/DriveCommand_itmoves.h>
#include <carolo/RemoteControl_itmoves.h>

const int FRONT_LEFT = 0;
const int FRONT_RIGHT = 1;
const int REAR_LEFT = 2;
const int REAR_RIGHT = 3;

class GazeboRosBodyControl {
public:
  GazeboRosBodyControl(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh),
    remote_steer_(4),
    remote_velocity_(0.0),
    autonomous_steer_(4),
    autonomous_velocity_(0.0),
    steer_(4),
    velocities_(4),
    rcmode_(0),
    manual_driving_(true),
    vel_amplification_factor_(20.0)
  {
    front_left_steering_pub_ = nh_.advertise<std_msgs::Float64>("front_left_steering_out", 5);
    front_left_velocity_pub_ = nh_.advertise<std_msgs::Float64>("front_left_velocity_out", 5);
    front_right_steering_pub_ = nh_.advertise<std_msgs::Float64>("front_right_steering_out", 5);
    front_right_velocity_pub_ = nh_.advertise<std_msgs::Float64>("front_right_velocity_out", 5);
    rear_left_steering_pub_ = nh_.advertise<std_msgs::Float64>("rear_left_steering_out", 5);
    rear_left_velocity_pub_ = nh_.advertise<std_msgs::Float64>("rear_left_velocity_out", 5);
    rear_right_steering_pub_ = nh_.advertise<std_msgs::Float64>("rear_right_steering_out", 5);
    rear_right_velocity_pub_ = nh_.advertise<std_msgs::Float64>("rear_right_velocity_out", 5);
    
    pnh_.getParam("vel_amplification_factor", vel_amplification_factor_);

    command_sub_rc_ = nh_.subscribe("canrc", 1, &GazeboRosBodyControl::canRemoteControlCB, this);
    command_sub_autonomous_ = nh_.subscribe("command_in_autonomous", 1, &GazeboRosBodyControl::driveCommandCB, this);
    command_sub_remote_ = nh_.subscribe("command_in", 1, &GazeboRosBodyControl::manualDrivingCB, this);
  }

  void remoteAutonomousSwitch()
  {
    if (rcmode_ == 0) {
       steer_[FRONT_LEFT] = remote_steer_[FRONT_LEFT];
       steer_[FRONT_RIGHT] = remote_steer_[FRONT_RIGHT];
       steer_[REAR_LEFT] = remote_steer_[REAR_LEFT];
       steer_[REAR_RIGHT] = remote_steer_[REAR_RIGHT];
       std::fill(velocities_.begin(), velocities_.end(), remote_velocity_);
    }
    else if (rcmode_ == 1) {
       steer_[FRONT_LEFT] = autonomous_steer_[FRONT_LEFT];
       steer_[FRONT_RIGHT] = autonomous_steer_[FRONT_RIGHT];
       steer_[REAR_LEFT] = autonomous_steer_[REAR_LEFT];
       steer_[REAR_RIGHT] = autonomous_steer_[REAR_RIGHT];
       std::fill(velocities_.begin(), velocities_.end(), autonomous_velocity_);
    }
    dynamicsControl();
  }

  void dynamicsControl()
  {
    ROS_DEBUG_STREAM("[GazeboRosBodyControl] Velocity "<<velocities_[FRONT_LEFT]);
    ROS_DEBUG_STREAM("[GazeboRosBodyControl] Steering front "<<steer_[FRONT_LEFT]);
    ROS_DEBUG_STREAM("[GazeboRosBodyControl] Steering rear "<<steer_[REAR_LEFT]);
    std_msgs::Float64 cmd_msg;
    cmd_msg.data = steer_[FRONT_LEFT];
    front_left_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[FRONT_LEFT];
    front_left_velocity_pub_.publish(cmd_msg);
    cmd_msg.data = steer_[FRONT_RIGHT];
    front_right_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[FRONT_RIGHT];
    front_right_velocity_pub_.publish(cmd_msg);
    cmd_msg.data = steer_[REAR_LEFT];
    rear_left_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[REAR_LEFT];
    rear_left_velocity_pub_.publish(cmd_msg);
    cmd_msg.data = steer_[REAR_RIGHT];
    rear_right_steering_pub_.publish(cmd_msg);
    cmd_msg.data = velocities_[REAR_RIGHT];
    rear_right_velocity_pub_.publish(cmd_msg);
  }

  void canRemoteControlCB(const std_msgs::UInt8::ConstPtr &msg) {
    rcmode_ = msg->data;
  }

  void manualDrivingCB(const carolo::RemoteControl_itmovesConstPtr &msg) {
    remote_steer_[FRONT_LEFT] = msg->steer_front;
    remote_steer_[FRONT_RIGHT] = msg->steer_front;
    remote_steer_[REAR_LEFT] = msg->steer_rear;
    remote_steer_[REAR_RIGHT] = msg->steer_rear;
    remote_velocity_ = msg->velocity*vel_amplification_factor_;
    remoteAutonomousSwitch();
  }

  void driveCommandCB(const carolo::DriveCommand_itmovesConstPtr &msg) {
    ROS_DEBUG_STREAM("Got msg front "<<msg->phi_f<<" and rear "<<msg->phi_r);
    autonomous_steer_[FRONT_LEFT] = -msg->phi_f;
    autonomous_steer_[FRONT_RIGHT] = -msg->phi_f;
    autonomous_steer_[REAR_LEFT] = -msg->phi_r;
    autonomous_steer_[REAR_RIGHT] = -msg->phi_r;
    autonomous_velocity_ = msg->lin_vel*vel_amplification_factor_;
    remoteAutonomousSwitch();
  }

private:
  ros::Publisher front_left_steering_pub_;
  ros::Publisher front_left_velocity_pub_;
  ros::Publisher front_right_steering_pub_;
  ros::Publisher front_right_velocity_pub_;
  ros::Publisher rear_left_steering_pub_;
  ros::Publisher rear_left_velocity_pub_;
  ros::Publisher rear_right_steering_pub_;
  ros::Publisher rear_right_velocity_pub_;

  ros::Subscriber command_sub_rc_;
  ros::Subscriber command_sub_autonomous_;
  ros::Subscriber command_sub_remote_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  bool manual_driving_;
  double vel_amplification_factor_;
  float remote_velocity_ ;
  float autonomous_velocity_ ;
  int rcmode_ ;

  std::vector<float> remote_steer_;
  std::vector<float> autonomous_steer_;
  std::vector<float> steer_;
  std::vector<float> velocities_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_gazebo_body_control_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  GazeboRosBodyControl body_control(nh, pnh);

#ifdef DEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}


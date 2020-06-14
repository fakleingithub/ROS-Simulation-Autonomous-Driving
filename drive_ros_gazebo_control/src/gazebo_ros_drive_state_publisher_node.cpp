#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <carolo/DriveState_itmoves.h>

const int FRONT_LEFT = 0;
const int FRONT_RIGHT = 1;
const int REAR_LEFT = 2;
const int REAR_RIGHT = 3;

int get_idx(const std::vector<std::string> &names, const std::string &name) {
  for (int i=0; i<names.size(); ++i) {
    if (names[i] == name)
      return i;
  }
  return -1;
}


class GazeboRosDriveStatePublisher {
public:
  GazeboRosDriveStatePublisher(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh),
    front_joint_name_("chassis_to_front_left_steer"),
    rear_joint_name_("chassis_to_rear_left_steer"),
    car_model_name_("cc_2019_car")
  {
    pnh_.getParam("front_steer_joint_name", front_joint_name_);
    pnh_.getParam("rear_steer_joint_name", rear_joint_name_);
    pnh_.getParam("car_model_name", car_model_name_);
    drive_state_pub_ = nh_.advertise<carolo::DriveState_itmoves>("drive_state_out", 5);
    joint_sub_ = nh_.subscribe("joint_state_in", 1, &GazeboRosDriveStatePublisher::jointStateCB, this);
    model_state_sub_ = nh_.subscribe("model_state_in", 1, &GazeboRosDriveStatePublisher::modelStateCB, this);
  }

  void jointStateCB(const sensor_msgs::JointStateConstPtr &joint_msg) {
    carolo::DriveState_itmoves msg_out;

    int front_joint_idx = get_idx(joint_msg->name, front_joint_name_);
    msg_out.steer_f = joint_msg->position[front_joint_idx];

    int rear_joint_idx = get_idx(joint_msg->name, rear_joint_name_);
    msg_out.steer_r = joint_msg->position[rear_joint_idx];

    msg_out.v = current_vel_;

    drive_state_pub_.publish(msg_out);
  }

  void modelStateCB(const gazebo_msgs::ModelStatesConstPtr &model_msg) {
    int car_model_idx = get_idx(model_msg->name, car_model_name_);
    current_vel_ = std::sqrt(std::pow(model_msg->twist[car_model_idx].linear.x, 2) +
                             std::pow(model_msg->twist[car_model_idx].linear.y, 2));
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher drive_state_pub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber model_state_sub_;

  double current_vel_;
  std::string front_joint_name_;
  std::string rear_joint_name_;
  std::string car_model_name_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_gazebo_drive_state_publisher_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  GazeboRosDriveStatePublisher drive_staet_publisher(nh, pnh);

#ifdef DEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "carolo/DriveCommand_itmoves.h"

class TwistToDriveStateAutonomous {
public:
    TwistToDriveStateAutonomous(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh){
        twist_sub_ = nh_.subscribe("twist_in_autonomous", 5, &TwistToDriveStateAutonomous::twistCB, this);
        drive_state_pub_ = nh_.advertise<carolo::DriveCommand_itmoves>("autonomous_control_out", 5);
    }
private:
    void twistCB(const geometry_msgs::TwistConstPtr& msg)
    {
        carolo::DriveCommand_itmoves msg_out;
        if (msg->linear.x != 0 || msg->linear.y != 0) {
            msg_out.phi_f = msg->angular.z;  //std::atan2(msg->linear.x, msg->linear.y);
            msg_out.phi_r = 0; //msg_out.phi_f;
            msg_out.lin_vel = msg->linear.x; //std::sqrt(std::pow(msg->linear.x, 2) + std::pow(msg->linear.y, 2));
        } 
        drive_state_pub_.publish(msg_out);
    }

    ros::Subscriber twist_sub_;
    ros::Publisher drive_state_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_ros_twist_to_autonmous");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TwistToDriveStateAutonomous drive_state(nh, pnh);

    while (ros::ok())
      ros::spin();

    return 0;
}

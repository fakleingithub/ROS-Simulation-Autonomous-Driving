#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "carolo/RemoteControl_itmoves.h"

class TwistToDriveState {
public:
    TwistToDriveState(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh){
        twist_sub_ = nh_.subscribe("twist_in", 5, &TwistToDriveState::twistCB, this);
        drive_state_pub_ = nh_.advertise<carolo::RemoteControl_itmoves>("remote_control_out", 5);
    }
private:
    void twistCB(const geometry_msgs::TwistConstPtr& msg)
    {
        // crab only without rotational inputs for now
        carolo::RemoteControl_itmoves msg_out;
        if (msg->linear.x != 0 || msg->linear.y != 0) {
            msg_out.steer_front = msg->angular.z;  //std::atan2(msg->linear.x, msg->linear.y);
            msg_out.steer_rear = 0; //msg_out.steer_front;
            msg_out.velocity = msg->linear.x; //std::sqrt(std::pow(msg->linear.x, 2) + std::pow(msg->linear.y, 2));
        } else {
            // to crab we would have to set our angles to different values on the wheels of the front and back,
            // not possible with the current uavcan msg structure
            //    / --- \
            //      | |
            //      | |
            //    \ --- /
            // should do something like this ideally but our current vel controller will probably fail here anyway
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
    ros::init(argc, argv, "gazebo_ros_twist_to_drivestate");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TwistToDriveState drive_state(nh, pnh);

    while (ros::ok())
      ros::spin();

    return 0;
}

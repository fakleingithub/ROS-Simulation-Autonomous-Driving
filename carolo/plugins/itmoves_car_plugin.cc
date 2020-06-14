#include <iostream>
#include <gazebo/gazebo.hh>
//#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>


#include <ignition/math/Vector3.hh>
#include <ignition/msgs/cmd_vel2d.pb.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

//#include <prius_msgs/Control.h>

#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/transport/AdvertiseOptions.hh>
#include <ignition/transport/Node.hh>

//#include "PriusHybridPlugin.hh"
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>

namespace gazebo {

class itmoves_car_plugin : public ModelPlugin {

	private:
	  event::ConnectionPtr updateConnection; // Pointer to the update event connection

		physics::ModelPtr model; // Pointer to the model
		physics::WorldPtr world;

		physics::JointPtr joint_wheel_fl;
		physics::JointPtr joint_wheel_fr;
		physics::JointPtr joint_wheel_rl;
		physics::JointPtr joint_wheel_rr;

		common::PID PID_steering;

		common::Time lastSimTime;
		double dt;

  public:
    itmoves_car_plugin() {
      int argc = 0;
      char *argv = nullptr;
      ros::init(argc, &argv, "PriusHybridPlugin");
      //ros::NodeHandle nh;
    }
      
  
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
			this->model = _model; // Store the pointer to the model
			this->world = this->model->GetWorld();

			this->joint_wheel_fl = this->model->GetJoint("itmoves_car::joint_wheel_fl");
			this->joint_wheel_fr = this->model->GetJoint("itmoves_car::joint_wheel_fr");
			this->joint_wheel_rl = this->model->GetJoint("itmoves_car::joint_wheel_rl");
			this->joint_wheel_rr = this->model->GetJoint("itmoves_car::joint_wheel_rr");

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&itmoves_car_plugin::OnUpdate, this)); // Listen to the update event. This event is broadcast every simulation iteration.

			/*this->PID_steering.SetPGain(0.1);
			this->PID_steering.SetIGain(0);
			this->PID_steering.SetDGain(0);

	    this->PID_steering.SetCmdMax(5);
	  	this->PID_steering.SetCmdMin(-5);*/
		}

		void OnUpdate() {// Called by the world update start event
      
			/*common::Time curTime = this->world->SimTime();
			this->dt = (curTime - this->lastSimTime).Double();
			this->lastSimTime = curTime;*/

			float torque_wheel_fl = 0.2;
			float torque_wheel_fr = 0.2;

			// impetus
			//this->joint_wheel_rl->SetForce(0, torque_wheel_fl);
			//this->joint_wheel_rr->SetForce(0, torque_wheel_fr);

			// steering
			//steering(0.1);
	    }

		void steering(double steeringAngleCmd) {
			/*double steeringAngle_wheelFl = this->joint_wheelFl->Position(0);
			double steeringAngle_wheelFr = this->joint_wheelFr->Position(0);

			printf("angle %f \n", steeringAngle_wheelFl);

			double deviation_wheelFl = steeringAngle_wheelFl - steeringAngleCmd;
			double deviation_wheelFr = steeringAngle_wheelFr - steeringAngleCmd;

			printf("deviation_wheelFl %f \n", deviation_wheelFl);

			double cmd_wheelFl = this->PID_steering.Update(deviation_wheelFl, this->dt);
			double cmd_wheelFr = this->PID_steering.Update(deviation_wheelFr, this->dt);


			printf("cmd_wheelFl %f \n", cmd_wheelFl);
			//printf("cmd_wheelFr %f \n", cmd_wheelFr);

			//double cmd_wheelFl = deviation_wheelFl*-10;
			//double cmd_wheelFr = deviation_wheelFr*-10;

			this->joint_wheelFl->SetForce(0, cmd_wheelFl);
			this->joint_wheelFr->SetForce(0, cmd_wheelFr);

			printf("\n");*/
		}
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(itmoves_car_plugin)
} // namespace gazebo



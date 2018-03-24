#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <string>     // std::string, std::stod
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"


namespace gazebo
{
class ModelPush : public ModelPlugin
{
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ModelPush::OnUpdate, this));
	
        sdf::ElementPtr e = _sdf->GetElement("vel");
        sdf::ParamPtr p  = e->GetValue();
        this->vel = std::stod(p->GetAsString());
        std::cout << "VALUE OF THE VEL FIELD" << this->vel << std::endl;
	
	int argc = 0;
	char** argv;
	ros::init(argc,argv,"gazebo_odom");
	ros::NodeHandle n;
	
	this->odompub = n.advertise<std_msgs::Float64>("odom_drifted", 1);
    }

    // Called by the world update start event
public: void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->vel));
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        gazebo::math::Pose pose = this->model->GetWorldPose();
        std::cout << "OnUpdate() called " << pose << std::endl;
	
	
	std_msgs::Float64 msg;
	msg.data = pose.pos.z;
	this->odompub.publish(msg);
    }

private: double vel;

private: ros::Publisher odompub;

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

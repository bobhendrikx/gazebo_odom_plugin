#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <string>     // std::string, std::stod
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <thread>
#include <time.h>
#include <chrono>
#include <tf/transform_broadcaster.h>


namespace gazebo
{
class OdomDriftPlugin : public ModelPlugin
{
 
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&OdomDriftPlugin::OnUpdate, this));
	
	
        sdf::ElementPtr e = _sdf->GetElement("vel");
        sdf::ParamPtr p  = e->GetValue();
        this->vel = std::stod(p->GetAsString());
        std::cout << "VALUE OF THE VEL FIELD" << this->vel << std::endl;
	
	int argc = 0;
	char** argv;
	ros::init(argc,argv,"gazebo_odom");
	ros::NodeHandle n;
	
	this->odompub = n.advertise<nav_msgs::Odometry>("odom_drifted", 1);
	
		
	//This thread object may not go out of scope say the C++ bosses
	this->thread = std::thread(&OdomDriftPlugin::asyncPublish,this);
    }

    // Called by the world update start event
public: void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        //this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->vel));
        //this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      
        this->pose = this->model->GetWorldPose();
        //std::cout << "OnUpdate() called " << pose << std::endl;
	
	nav_msgs::Odometry odom;
	
	odom.pose.pose.orientation.x = 1;
	odom.pose.pose.orientation.y = 1;
	odom.pose.pose.orientation.z = 1;
	
	
	std_msgs::Float64 msg;
	//msg.data = pose.pos.z;
	//odompub.publish(msg);
    }
    
public: void asyncPublish()
{
  while(ros::ok())
  {
  std::cout << "x: " << this->pose.pos.x << "  y: " << this->pose.pos.y << "  a: " << this->pose.rot.x << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  nav_msgs::Odometry odom;
  
  odom.pose.pose.position.x = this->pose.pos.x;
  odom.pose.pose.position.y = this->pose.pos.y;
  odom.pose.pose.orientation.z = this->pose.rot.z;
  
  //odom.twist.twist.linear.x = 
  //odom.twist.twist.linear.y = 
  //odom.twist.twist.angular.z = 
  this->odompub.publish(odom);
  
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(this->pose.pos.x,this->pose.pos.y,0));
  tf::Quaternion q;
  q.setRPY(0,0,this->pose.rot.z);
  transform.setRotation(q);
  
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","baselink"));
  
  //std_msgs::Float64 msg;
  //msg.data = 1; //pose.pos.z;
  //this->odompub.publish(msg);
  
  }
}

public: gazebo::math::Pose pose;
 
private: std::thread thread;

private: double vel;

//private: static ros::Rate rate;

private: ros::Publisher odompub;

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(OdomDriftPlugin)
}

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
	
	//Initialize the drifted odom transform to zero
	this->odomdrifted.x = 0;
	this->odomdrifted.y = 0;
	this->odomdrifted.z = 0;
	
	//Initialize odom in previous timestep to zero
	this->truepose_old.x = 0;
	this->truepose_old.y = 0;
	this->truepose_old.z = 0;
	
		
	//This thread object may not go out of scope say the C++ bosses
	this->thread = std::thread(&OdomDriftPlugin::asyncPublish,this);
    }

    // Called by the world update start event
public: void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        //this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->vel));
        //this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      
	this->mu.lock();
        this->pose = this->model->GetWorldPose();
	this->mu.unlock();
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
  double deltarot1;
  double deltarot2;
  double deltatrans;
  double dr1;
  double dr2;
  double dtr;
  
  while(ros::ok())
  {
  //std::cout << "x: " << this->pose.pos.x << "  y: " << this->pose.pos.y << "  a: " << this->pose.rot.GetYaw() << std::endl;
  //std::cout << "x_old: " << this->truepose_old.x << "  y_old: " << this->truepose_old.y << "  a_old: " << this->truepose_old.z << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  this->mu.lock();
  
  nav_msgs::Odometry odom;
  
  
  deltarot1 = 0;
  //Calculate odometry difference
  // Mind the small positive number, it causes the odom to flip because suddenly angles of pi are registered
  std::cout << " epsilon =  " << std::sqrt(std::pow(this->pose.pos.x - truepose_old.x,2)+ std::pow(this->pose.pos.y - truepose_old.y,2)) << std::endl;
  if( std::sqrt(std::pow(this->pose.pos.x - truepose_old.x,2)+ std::pow(this->pose.pos.y - truepose_old.y,2)) > 0.001){
   deltarot1 = std::atan2(this->pose.pos.y - truepose_old.y,this->pose.pos.x - this->truepose_old.x) - truepose_old.z;
   deltatrans = std::sqrt(std::pow(this->pose.pos.x - truepose_old.x,2)+ std::pow(this->pose.pos.y - truepose_old.y,2));
  }
  else{
   deltarot1 = 0;
   deltatrans = 0;
  }
  deltarot2 = this->pose.rot.GetYaw() - deltarot1 - truepose_old.z;
  
  std::cout << "dr1: " << deltarot1 << "  dr2: " << deltarot2 << "  dtr: " << deltatrans << std::endl;
  
  double noise = 0.0;
  //Add noise to delta values
  dr1 = deltarot1 + 0.002*std::abs(deltarot1);
  dr2 = deltarot2 + noise + 0.002*std::abs(deltatrans) ;
  dtr = deltatrans + 0.01*std::abs(deltatrans) + 0.002*std::abs(deltarot1);
  
  this->odomdrifted.x = std::cos(this->odomdrifted.z + dr1) * dtr + this->odomdrifted.x;  
  this->odomdrifted.y = std::sin(this->odomdrifted.z + dr1) * dtr + this->odomdrifted.y;    
  this->odomdrifted.z = this->odomdrifted.z + dr1 + dr2; 
 
  odom.pose.pose.position.x =  this->odomdrifted.x;
  odom.pose.pose.position.y =  this->odomdrifted.y; 
  odom.pose.pose.orientation.z =    this->odomdrifted.z;                        
  
  this->odompub.publish(odom);
  
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,0));
  tf::Quaternion q;
  q.setRPY(0,0,odom.pose.pose.orientation.z);
  transform.setRotation(q);
  
  std::cout << "x: " << odom.pose.pose.position.x << "  y: " << odom.pose.pose.position.y << "  a: " << odom.pose.pose.orientation.z << std::endl;
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link_drifted"));
  
  //std_msgs::Float64 msg;
  //msg.data = 1; //pose.pos.z;
  //this->odompub.publish(msg);
  
  this->truepose_old.x = this->pose.pos.x;
  this->truepose_old.y = this->pose.pos.y;
  this->truepose_old.z = this->pose.rot.GetYaw();  
  
  this->mu.unlock();
  }
}

private: struct odomdrift {double x; double y; double z;} odomdrifted;
private: struct odomold {double x; double y; double z;} truepose_old;

public: gazebo::math::Pose pose;
 
private: std::thread thread;

private: double vel;
private: std::mutex mu;

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

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <string>     // std::string, std::stod


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
    }

    // Called by the world update start event
public: void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->vel));
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0.1));
        gazebo::math::Pose pose = this->model->GetWorldPose();
        std::cout << "OnUpdate() called " << pose << std::endl;
    }

private: double vel;

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

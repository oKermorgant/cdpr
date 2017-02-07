
#include <ros/ros.h>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>

#include <cdpr/cdpr_plugin.h>

using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

void CDPRPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get model and name
    model_ = _model;

    // register ROS node & time
    rosnode_ = ros::NodeHandle();
    t_prev_ = 0;

    // *** JOINT CONTROL
    joints_.clear();

    if(model_->GetJointCount() != 0)
    {
        // initialize subscriber to joint commands
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    "cable_command", 1,
                    boost::bind(&CDPRPlugin::JointCommandCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        joint_command_subscriber_ = rosnode_.subscribe(ops);
        joint_command_received_ = false;

        // setup joint states
        std::vector<std::string> joint_names;
        std::string name;
        physics::JointPtr joint;

        for(unsigned int i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();

            if(name.find("cable") == 0) // we got a cable
            {
                joints_.push_back(joint);
                // save name
                joint_names.push_back(name);                
            }
        }

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("cable_states", 1);
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
        joint_states_.effort.resize(joints_.size());
    }
    // *** END JOINT CONTROL

    // store update rate
    if(_sdf->HasElement("updateRate"))
        update_T_ = 1./_sdf->Get<double>("updateRate");
    else
        update_T_ = 0;

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CDPRPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started CDPR Plugin for %s.", _model->GetName().c_str());
}

void CDPRPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();

    if(controller_is_running_)
    {
        // deal with joint control
        if(joint_command_received_)
        {
            physics::JointPtr joint;
            unsigned int idx;
            for(unsigned int i=0;i<joint_command_.name.size();++i)
            {
                // find corresponding model joint
                idx = std::distance(joint_states_.name.begin(), std::find(joint_states_.name.begin(), joint_states_.name.end(), joint_command_.name[i]));
                joint = joints_[idx];
                joint->SetForce(0,joint_command_.effort[i]);
            }
        }
    }

    // publish joint states
    double t = ros::Time::now().toSec();
    if((t-t_prev_) > update_T_ && joints_.size() != 0)
    {
        t_prev_ = t;
        joint_states_.header.stamp = ros::Time::now();

        for(unsigned int i=0;i<joints_.size();++i)
        {
            joint_states_.position[i] = joints_[i]->GetAngle(0).Radian();
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
            joint_states_.effort[i] = joints_[i]->GetForce(0);
        }
        joint_state_publisher_.publish(joint_states_);
    }
}



}   // namespace gazebo

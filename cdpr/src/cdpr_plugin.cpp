
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
    robot_namespace_ = model_->GetName();
    controller_is_running_ = true;

    // register ROS node & time
    rosnode_ = ros::NodeHandle(robot_namespace_);
    ros::NodeHandle control_node(rosnode_, "controllers");
    t_prev_ = 0;

    // *** JOINT CONTROL
    joints_.clear();

    if(model_->GetJointCount() != 0)
    {
        std::string joint_command_topic, joint_state_topic;
        control_node.param("config/joints/command", joint_command_topic, std::string("joint_command"));
        control_node.param("config/joints/state", joint_state_topic, std::string("joint_state"));

        // initialize subscriber to joint commands
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    joint_command_topic, 1,
                    boost::bind(&CDPRPlugin::JointCommandCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        joint_command_subscriber_ = rosnode_.subscribe(ops);
        joint_command_received_ = false;

        // write joint limits and setup joint states
        std::vector<std::string> joint_names;
        std::vector<double> joint_min, joint_max, vel_max;
        std::string name;
        physics::JointPtr joint;
        bool cascaded_position = true;
        if(control_node.hasParam("config/joints/cascaded_position"))
            control_node.getParam("config/joints/cascaded_position", cascaded_position);


        char param[256];
        for(unsigned int i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();

            if(control_node.hasParam(name))
            {
                joints_.push_back(joint);
                // set max velocity or max effort for the position PID
                sprintf(param, "%s/position/i_clamp", name.c_str());
                if(cascaded_position)
                    control_node.setParam(param, joint->GetVelocityLimit(0));
                else
                    control_node.setParam(param, joint->GetEffortLimit(0));

                // set max effort for the velocity PID
                sprintf(param, "%s/velocity/i_clamp", name.c_str());
                control_node.setParam(param, joint->GetEffortLimit(0));

                // set antiwindup to true - why would anyone set it to false?
                sprintf(param, "%s/position/antiwindup", name.c_str());
                control_node.setParam(param, true);
                sprintf(param, "%s/velocity/antiwindup", name.c_str());
                control_node.setParam(param, true);

                // save name and joint limits
                joint_names.push_back(name);
                joint_min.push_back(joint->GetLowerLimit(0).Radian());
                joint_max.push_back(joint->GetUpperLimit(0).Radian());
                vel_max.push_back(joint->GetVelocityLimit(0));
            }
        }

        // push setpoint topic, name, lower and bound
        control_node.setParam("config/joints/name", joint_names);
        control_node.setParam("config/joints/lower", joint_min);
        control_node.setParam("config/joints/upper", joint_max);
        control_node.setParam("config/joints/velocity", vel_max);

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
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

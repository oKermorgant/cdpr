#ifndef CDPR_PLUGIN_H
#define CDPR_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>

namespace gazebo
{

class CDPRPlugin : public ModelPlugin
{
public:
    CDPRPlugin() {}
    ~CDPRPlugin()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_event_);
        rosnode_.shutdown();
        //    delete rosnode_;
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:

    // parse received joint command (joint states)
    void JointCommandCallBack(const sensor_msgs::JointStateConstPtr &_msg)
    {
        // store received joint state
        joint_command_ = *_msg;
        joint_command_received_ = true;
    }

private:
    // -- general data ----------------------------------------
    ros::NodeHandle rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_event_;
    double update_T_;

    // -- joint control ----------------------------------------
    // model joint data
    std::vector<physics::JointPtr> joints_;
    double f_max;

    // subscriber
    ros::Subscriber joint_command_subscriber_;
    sensor_msgs::JointState joint_command_;
    bool joint_command_received_;

    // -- publishers ----------------------------------------

    // publisher to joint state
    ros::Publisher joint_state_publisher_;
    sensor_msgs::JointState joint_states_;
    double t_prev_;

    // publisher of platform position
    ros::Publisher pf_publisher_;
    gazebo_msgs::LinkState pf_state_;
    physics::LinkPtr frame_link_, platform_link_;
};
GZ_REGISTER_MODEL_PLUGIN(CDPRPlugin)
}
#endif // CDPR_PLUGIN_H

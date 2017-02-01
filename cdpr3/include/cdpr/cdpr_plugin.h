#ifndef CABLEDRIVENPARALLELROBOT_H
#define CABLEDRIVENPARALLELROBOT_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

namespace gazebo
{

class CableDrivenParallelRobotPlugin : public ModelPlugin
{
public:
    CableDrivenParallelRobotPlugin() {}
    ~CableDrivenParallelRobotPlugin()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_event_);
        rosnode_.shutdown();
        //    delete rosnode_;
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    void ReadVector3(const std::string &_string, math::Vector3 &_vector);

};
GZ_REGISTER_MODEL_PLUGIN(CableDrivenParallelRobotPlugin)
}
#endif // CABLEDRIVENPARALLELROBOT_H

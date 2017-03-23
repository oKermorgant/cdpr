#ifndef workspace_H
#define workspace_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <visp/vpHomogeneousMatrix.h>

using std::endl;
using std::cout;

class Trajectory
{
public:
    Trajectory(ros::NodeHandle &w_node)
    {   
       /* // publisher to setpoints
        setpointPose_pub = w_node.advertise<geometry_msgs::Pose>("pf_setpoint", 1);
        setpointVel_pub = w_node.advertise<geometry_msgs::Twist>("desired_vel",1);
        setpointAcc_pub = w_node.advertise<geometry_msgs::Twist>("desired_acc",1);*/

        // load model parameters
        ros::NodeHandle model(w_node, "model");

        model.getParam("platform/mass",mass_);

        // initialize the size of the basic frame
        XmlRpc::XmlRpcValue element;
        model.getParam("points", element);
        n_cable = element.size();
        double x, y, z;
        for(unsigned int i=0;i<n_cable;++i)
        {
            x = element[i]["frame"][0];
            y = element[i]["frame"][1];
            z = element[i]["frame"][2];
            Pf.push_back(vpTranslationVector(x, y, z));
            /*x = element[i]["platform"][0];
            y= element[i]["platform"][1];
            z = element[i]["platform"][2];
            Pp.push_back(vpTranslationVector(x, y, z));*/
        }
        model.getParam("platform/size", element);
        size_pf.resize(3);

        for(unsigned int i=0;i<3;++i)
           size_pf[i] = element[i];
   
    }

    //inline void InitializeTime(double &t_i, double &t_f) {t_i = t0; t_f = t4;}
    inline void getBoundary(vpMatrix &b_i){b_i= bi;}
    //inline void InitializePose(vpRowVector &x_i, vpRowVector &x_f) {x_i = xi; x_f = xf;}
    //inline void InitializeGain(double &t_i, double &t_f) {t_i = Kp; t_f = Kd;}



    void sendDesiredpara(vpColVector p, vpColVector v, vpColVector acc)
    {
        // write effort to jointstate
          
        pf_d.position.x=p[0],pf_d.position.y=p[1],pf_d.position.z=p[2];
        pf_d.orientation.w=1;
        vel_d.linear.x=v[0],vel_d.linear.y=v[1],vel_d.linear.z=v[2];
        acc_d.linear.x=acc[0],acc_d.linear.y=acc[1],acc_d.linear.z=acc[2];

        setpointPose_pub.publish(pf_d);
        setpointVel_pub.publish(vel_d); 
        setpointAcc_pub.publish(acc_d);
    }

    // publisher to the desired pose , volecity and acceleration
   protected:
    ros::Publisher setpointPose_pub, setpointVel_pub, setpointAcc_pub;
    geometry_msgs::Twist vel_d, acc_d; 
    geometry_msgs::Pose pf_d;

    // model parameter
    double mass_;
    vpColVector  size_pf;
    std::vector<vpTranslationVector> Pf, Pp;

};

#endif // workspace_H

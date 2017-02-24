#ifndef trajectory_H
#define trajectory_H

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
    Trajectory(ros::NodeHandle &_node)
    {   
        // publisher to setpoints
        setpointPose_pub = _node.advertise<geometry_msgs::Pose>("pf_setpoint", 1);
        setpointVel_pub = _node.advertise<geometry_msgs::Twist>("desired_vel",1);
        setpointAcc_pub = _node.advertise<geometry_msgs::Twist>("desired_acc",1);

        // load model parameters
        ros::NodeHandle Tra(_node, "Tra");


        Tra.getParam("time/initial", ti);
        Tra.getParam("time/finial", tf);

        XmlRpc::XmlRpcValue element_i,element_f;
        Tra.getParam("position/initial", element_i);
        Tra.getParam("position/finial", element_f);
        xi.resize(3);
        xf.resize(3);
        for (unsigned int i=0;i<3;++i)
        {
            xi[i]=element_i[i];
            xf[i]=element_f[i];
        }

    }

    inline void InitializeTime(double &t_i, double &t_f) {t_i = ti; t_f = tf;}
    inline void InitializePose(vpRowVector &x_i, vpRowVector &x_f) {x_i = xi; x_f = xf;}

    inline vpMatrix getLmatrix(double &t)
    {
        vpMatrix M;
        M.resize(6,6);
        M[0][5]=1; M[1][4]=1; M[2][3]=2;
        M[3][0]=t*t*t*t*t; M[3][1]=t*t*t*t; M[3][2]=t*t*t; M[3][3]=t*t; M[3][4]=t; M[3][5]=1;
        M[4][0]=5*t*t*t*t; M[4][1]=4*t*t*t; M[4][2]=3*t*t; M[4][3]=2*t; M[4][4]=1; 
        M[5][0]=20*t*t*t; M[5][1]=12*t*t; M[5][2]=6*t; M[5][3]=2;  
        return M;
    }
    // get the desired position xyz
    inline vpRowVector getposition(double t, vpMatrix a)
    { 
        vpRowVector l;
        l.resize(6);
        l[0]=t*t*t*t*t; l[1]=t*t*t*t; l[2]=t*t*t; l[3]=t*t; l[4]=t; l[5]=1;
        vpRowVector position=l*a;
        return position;
    }
    //get the desired velocity
    inline vpRowVector getvelocity(double t, vpMatrix a)
    { 
        vpRowVector l;
        l.resize(6);
         l[0]=5*t*t*t*t; l[1]=4*t*t*t; l[2]=3*t*t; l[3]=2*t; l[4]=1; 
        vpRowVector vel=l*a;
        return vel;
    }

    inline vpRowVector getacceleration(double t, vpMatrix a)
    { 
        vpRowVector l;
        l.resize(6);
        l[0]=20*t*t*t; l[1]=12*t*t; l[2]=6*t; l[3]=2;  
        vpRowVector acc=l*a;
        return acc;
    }

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

   protected:
    // publisher to the desired pose , volecity and acceleration
    ros::Publisher setpointPose_pub, setpointVel_pub, setpointAcc_pub;
    geometry_msgs::Twist vel_d, acc_d; 
    geometry_msgs::Pose pf_d;

    // model parameter
    double ti, tf;
    vpRowVector  xi, xf;

};

#endif // trajectory_H

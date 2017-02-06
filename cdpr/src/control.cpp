
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/JointState.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>


#include <visp/vpHomogeneousMatrix.h>

using namespace std;
using namespace gazebo;

void Param(ros::NodeHandle &nh, const string &key, double &val)
{
    if(nh.hasParam(key))
        nh.getParam(key, val);
    else
        nh.setParam(key, val);
}


class Listener
{
public:
    Listener(ros::NodeHandle &_nh)
    {
        // init listener to gazebo link states
        gz_sub_ = _nh.subscribe("/gazebo/link_states", 1, &Listener::LinkStates_cb, this);
        gz_received_ = false;

        // init listener to pose setpoint
        setpoint_sub_ = _nh.subscribe("pf_setpoint", 1, &Listener::Setpoint_cb, this);
        setpoint_received_ = false;


    }

    // callback for link states
    void LinkStates_cb(const gazebo_msgs::LinkStatesConstPtr &_msg)
    {
        gz_received_ = true;
        vpHomogeneousMatrix wMf;
        for(unsigned int i=0;i<_msg->name.size();++i)
        {
            if(_msg->name[i] == "cube::platform")
            {
                fMp_.insert(vpTranslationVector(_msg->pose[i].position.x, _msg->pose[i].position.y, _msg->pose[i].position.z));
                fMp_.insert(vpQuaternionVector(_msg->pose[i].orientation.x, _msg->pose[i].orientation.y, _msg->pose[i].orientation.z,_msg->pose[i].orientation.w));
            }
            else if(_msg->name[i] == "cube::frame")
            {
                wMf.insert(vpTranslationVector(_msg->pose[i].position.x, _msg->pose[i].position.y, _msg->pose[i].position.z));
                wMf.insert(vpQuaternionVector(_msg->pose[i].orientation.x, _msg->pose[i].orientation.y, _msg->pose[i].orientation.z,_msg->pose[i].orientation.w));
            }
        }
        fMp_ = wMf.inverse() * fMp_;
    }


    // callback for pose setpoint
    void Setpoint_cb(const geometry_msgs::PoseConstPtr &_msg)
    {
        setpoint_received_ = true;
        fMpd_.insert(vpTranslationVector(_msg->position.x, _msg->position.y, _msg->position.z));
        fMpd_.insert(vpQuaternionVector(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z,_msg->orientation.w));
    }


    bool setpoint_received_, gz_received_;
    vpHomogeneousMatrix fMp_, fMpd_;

protected:
    ros::Subscriber gz_sub_, setpoint_sub_;

};



int main(int argc, char ** argv)
{


    cout.precision(3);
    // init ROS node
    ros::init(argc, argv, "cdpr_control");
    ros::NodeHandle nh;


    // load model parameters
    ros::NodeHandle model(nh, "model");

    // platform mass
    double mass;
    model.getParam("platform/mass", mass);

    // platform initial pose
    vector<double> xyz, rpy;
    model.getParam("platform/position/xyz", xyz);
    model.getParam("platform/position/rpy", rpy);
    vpHomogeneousMatrix fMp, fMpd;
    vpRxyzVector r(rpy[0], rpy[1], rpy[2]);
    fMpd.insert(vpRotationMatrix(r));
    fMpd.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

    // cable min / max
    double u_min, u_max;
    model.getParam("joints/actuated/effort", u_max);
    model.getParam("joints/actuated/min", u_min);

    // cable attach points
    XmlRpc::XmlRpcValue points;
    model.getParam("points", points);
    const unsigned int n = points.size();
    vector<vpTranslationVector> fPf, pPp;
    double x, y, z;
    for(unsigned int i=0;i<n;++i)
    {
        x = points[i]["frame"][0];
        y= points[i]["frame"][1];
        z = points[i]["frame"][2];
        fPf.push_back(vpTranslationVector(x, y, z));
        x = points[i]["platform"][0];
        y= points[i]["platform"][1];
        z = points[i]["platform"][2];
        pPp.push_back(vpTranslationVector(x, y, z));
    }

    // publisher to cable tensions
    ros::Publisher cable_pub = nh.advertise<sensor_msgs::JointState>("cable_command", 1);
    sensor_msgs::JointState cable_sp;
    char cable_name[256];
    for(unsigned int i=0;i<n;++i)
    {
        sprintf(cable_name, "cable%i", i);
        cable_sp.name.push_back(string(cable_name));
    }
    cable_sp.effort.resize(n);

    Listener listener(nh);

    vpMatrix T(6, n);   // tau = T.u + g
    vpColVector g(6), w, tau, u, tau_i(6), u0(n);
    g[2] = -mass * 9.81;
    vpMatrix fRRp(6,6);

    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpColVector v(6);
    vpTranslationVector fTp, f;
    vpRotationMatrix fRp;

    // gain
    double Kp = 0.1, Ki = 0.01;
    Param(nh, "Kp", Kp);
    Param(nh, "Ki", Ki);

    // solver
    /*
    okSolveQP qp;
    vpMatrix qp_C(2*n, n);
    vpColVector qp_d(2*n), qp_r(6);
    for(unsigned int i=0;i<n;++i)
    {
        qp_d[i] = -u_min;
        qp_d[i+n] = u_max;
        qp_C[i][i] = -1;
        qp_C[i+n][i] = 1;
    }
    qp.addInequality(qp_C, qp_d, 0);
    qp.addEquality(T, qp_r, 1);*/

    cout << "CDPR control ready" << fixed << endl;

    while(ros::ok())
    {
        cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        nh.getParam("Ki", Ki);
        if(listener.gz_received_)
        {
            // current position
            fMp = listener.fMp_;
            fMp.extract(fTp);
            fMp.extract(fRp);

            // desired position
            if(listener.setpoint_received_)
                fMpd = listener.fMpd_;

            // position error in platform frame
            tau = vpPoseVector(fMp.inverse()*fMpd);
            //tau = vpExponentialMap::inverse(M.inverse()*Md.inverse());
            cout << "Position error in platform frame: " << tau.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    fRRp[i][j] = fRRp[i+3][j+3] = fRp[i][j];

            // position error in fixed frame
            tau = fRRp * tau;
            // PI controller to wrench in fixed frame
            tau_i += tau * dt;
            tau = Kp * (tau + Ki*tau_i);
            cout << "Desired wrench in platform frame: " << (fRRp.transpose()*(tau - g)).t() << fixed << endl;

            // build T matrix depending on current attach points
            for(unsigned int i=0;i<n;++i)
            {
                // vector between platform point and frame point in platform frame
                f = fRp.t() * (fPf[i] - fTp) + pPp[i];
                f /= f.euclideanNorm();
                // corresponding force in platform frame
                w = pPp[i].skew() * f;
                for(unsigned int k=0;k<3;++k)
                {
                    T[k][i] = f[k];
                    T[k+3][i] = w[k];
                }
            }

            // solve problem : tau = T.u + g
            //qp_r = fRRp.transpose() * (tau-g);
            //qp.solveCascade(u);
            u = T.pseudoInverse() * fRRp.transpose()* (tau - g);
            cout << "   Verif T.u+g in platform frame: " << (T*u).t() << fixed << endl;
            //u = 0.5*(u+u0);
            //u0 = u;

            // write effort to jointstate
            for(unsigned int i=0;i<n;++i)
            {
                cable_sp.effort[i] = u[i];
            }
            cable_sp.header.stamp = ros::Time::now();
            cable_pub.publish(cable_sp);

            cout << "Sending tensions: " << u.t() << fixed << endl;
        }

        ros::spinOnce();
        loop.sleep();
    }





}

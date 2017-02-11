#include <cdpr/cdpr.h>

using std::endl;
using std::cout;

CDPR::CDPR(ros::NodeHandle &_nh)
{

    // init listener to platform state
    platform_sub = _nh.subscribe("pf_state", 1, &CDPR::PFState_cb, this);
    platform_ok = false;

    // init listener to pose setpoint
    setpoint_sub = _nh.subscribe("pf_setpoint", 1, &CDPR::Setpoint_cb, this);

    // init listener to cable states
    cables_sub = _nh.subscribe("cable_states", 1, &CDPR::Cables_cb, this);
    cables_ok = false;

    // load model parameters
    ros::NodeHandle model(_nh, "model");
    model.getParam("platform/mass", mass_);

    // inertia matrix
    XmlRpc::XmlRpcValue element;
    model.getParam("platform/inertia", element);
    inertia_.resize(3,3);
    for(unsigned int i=0;i<3;++i)
        inertia_[i][i] = element[i];
    inertia_[0][1] = inertia_[1][0] = element[3];
    inertia_[0][2] = inertia_[2][0] = element[4];
    inertia_[2][1] = inertia_[1][2] = element[5];

    // cable min / max
    model.getParam("joints/actuated/effort", f_max);
    model.getParam("joints/actuated/min", f_min);

    // cable attach points    
    model.getParam("points", element);
    n_cable = element.size();
    double x, y, z;
    for(unsigned int i=0;i<n_cable;++i)
    {
        x = element[i]["frame"][0];
        y = element[i]["frame"][1];
        z = element[i]["frame"][2];
        Pf.push_back(vpTranslationVector(x, y, z));
        x = element[i]["platform"][0];
        y= element[i]["platform"][1];
        z = element[i]["platform"][2];
        Pp.push_back(vpTranslationVector(x, y, z));
    }

    // initial desired pose = home
    std::vector<double> xyz, rpy;
    model.getParam("platform/position/xyz", xyz);
    model.getParam("platform/position/rpy", rpy);
    vpRxyzVector r(rpy[0], rpy[1], rpy[2]);
    Md_.insert(vpRotationMatrix(r));
    Md_.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));


    // publisher to cable tensions
    tensions_pub = _nh.advertise<sensor_msgs::JointState>("cable_command", 1);
    char cable_name[256];
    for(unsigned int i=0;i<n_cable;++i)
    {
        sprintf(cable_name, "cable%i", i);
        tensions_msg.name.push_back(std::string(cable_name));
    }
    tensions_msg.effort.resize(n_cable);
}


void CDPR::computeW(vpMatrix &W)
{
    // build W matrix depending on current attach points
    vpTranslationVector T;  M_.extract(T);
    vpRotationMatrix R;     M_.extract(R);

    vpTranslationVector f;
    vpColVector w;
    for(unsigned int i=0;i<n_cable;++i)
    {
        // vector between platform point and frame point in platform frame
        f = R.t() * (Pf[i] - T) + Pp[i];
        f /= f.euclideanNorm();
        // corresponding force in platform frame
        w = Pp[i].skew() * f;
        for(unsigned int k=0;k<3;++k)
        {
            W[k][i] = f[k];
            W[k+3][i] = w[k];
        }
    }
}


void CDPR::sendTensions(vpColVector &f)
{
    // write effort to jointstate
    for(unsigned int i=0;i<n_cable;++i)
        tensions_msg.effort[i] = f[i];
    tensions_msg.header.stamp = ros::Time::now();

    tensions_pub.publish(tensions_msg);
}






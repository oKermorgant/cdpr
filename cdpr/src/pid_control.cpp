
#include <cdpr/cdpr.h>

using namespace std;


/*
 * Basic PID controller to show input/output of the CDPR class
 *
 * Does not consider positive-only cable tensions and assumes it is more or less a UPS parallel robot
 *
 *
 */


void Param(ros::NodeHandle &nh, const string &key, double &val)
{
    if(nh.hasParam(key))
        nh.getParam(key, val);
    else
        nh.setParam(key, val);
}

int main(int argc, char ** argv)
{

    cout.precision(3);
    // init ROS node
    ros::init(argc, argv, "cdpr_control");
    ros::NodeHandle nh;

    // init CDPR class from parameter server
    CDPR robot(nh);
    const unsigned int n = robot.n_cables();

    vpMatrix W(6, n);   // tau = W.u + g
    vpColVector g(6), tau, u, tau_i(6);
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6);

    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M, Md;
    vpRotationMatrix R;

    // gain
    double Kp = 0.1, Ki = 0.01;
    Param(nh, "Kp", Kp);
    Param(nh, "Ki", Ki);

    cout << "CDPR control ready" << fixed << endl;

    while(ros::ok())
    {
        cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        nh.getParam("Ki", Ki);

        if(robot.ok())  // messages have been received
        {
            // current position
            robot.getPose(M);
            M.extract(R);

            // desired position
            robot.getDesiredPose(Md);

            // position error in platform frame
            tau = vpPoseVector(M.inverse()*Md);
            cout << "Position error in platform frame: " << tau.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];

            // position error in fixed frame
            tau = RR * tau;
            // PI controller to wrench in fixed frame
            tau_i += tau * dt;
            tau = Kp * (tau + Ki*tau_i);
            cout << "Desired wrench in platform frame: " << (RR.transpose()*(tau - g)).t() << fixed << endl;

            // build W matrix depending on current attach points
            robot.computeW(W);

            u = W.pseudoInverse() * RR.transpose()* (tau - g);
            cout << "Checking W.u+g in platform frame: " << (W*u).t() << fixed << endl;

            // send tensions
            robot.sendTensions(u);
        }

        ros::spinOnce();
        loop.sleep();
    }





}

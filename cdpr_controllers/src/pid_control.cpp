
#include <ros/ros.h>
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
    vpColVector g(6), tau(6), err, f, err_i(6), err0(6);
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6);

    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M;
    vpRotationMatrix R;

    // gain
    double Kp = 1000, Ki = 5, Kd = 20;  // tuned for Caroca
    Param(nh, "Kp", Kp);
    Param(nh, "Ki", Ki);
    Param(nh, "Kd", Kd);

    cout << "CDPR control ready" << fixed << endl;

    while(ros::ok())
    {
        cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        nh.getParam("Ki", Ki);
        nh.getParam("Kd", Kd);

        if(robot.ok())  // messages have been received
        {
            // current position
            robot.getPose(M);
            M.extract(R);

            // position error in platform frame
            err = robot.getPoseError();
            cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];

            // position error in fixed frame
            err = RR * err;
            // I term to wrench in fixed frame
            for(unsigned int i=0;i<6;++i)
                if(tau[i] < robot.mass()*9.81)
                    err_i[i] += err[i] * dt;

            tau = Kp * (err + Ki*err_i);

            // D term?
            if(err0.infinityNorm())
               tau += Kp * Kd * (err - err0)/dt;
            err0 = err;
            cout << "Desired wrench in platform frame: " << (RR.transpose()*(tau - g)).t() << fixed << endl;

            // build W matrix depending on current attach points
            robot.computeW(W);

            f = W.pseudoInverse() * RR.transpose()* (tau - g);
            cout << "Checking W.f+g in platform frame: " << (W*f).t() << fixed << endl;
            cout << "sending tensions: " << f.t() << endl;

            // send tensions
            robot.sendTensions(f);
        }

        ros::spinOnce();
        loop.sleep();
    }





}


#include <ros/ros.h>
#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>

using namespace std;


/*
 * Basic PID controller to show input/output of the CDPR class
 *
 * Does not consider dynamics except for gravity
 * Actual TDA depends on "control" parameter
 *
 */

typedef enum
{
    minA, minW, minT, noMin
} minType;


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
    ros::NodeHandle nh, nh_priv("~");

    // init CDPR class from parameter server
    CDPR robot(nh);
    const unsigned int n = robot.n_cables();

    // get control type
    minType control = noMin;
    /* std::string control_type;
    nh_priv.getParam("control", control_type);
    if(control_type == "noMin")
        control = noMin;
    else if(control_type == "minA")
        control = minA;
    else if(control_type == "minT")
        control = minT;*/

    double tauMin, tauMax;
    robot.tensionMinMax(tauMin, tauMax);

    // QP variables
    vpColVector x(n);
    vpMatrix Q, A, C, W(6,n);
    vpColVector r, b, d, w(6);

    if(control == minT)
    {
        // min |tau|
        //  st W.tau = w
        //  st t- < tau < tau+

        // min tau
        Q.eye(n);
        r.resize(n);
        // equality constraint
        A.resize(6,n);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i+n] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minW)
    {
        // min |W.tau - w|
        //   st t- < tau < t+

        Q.resize(6,n);
        r.resize(6);
        // no equality constraints
        A.resize(0,n);
        b.resize(0);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i+n] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minA)
    {
        // min |tau| - alpha
        //  st W.tau = alpha.w
        //  st 0 < alpha < 1
        //  st t- < tau < t+
        x.resize(n+1);

        Q.eye(7);
        r.resize(7);r[6] = 1;
        // equality constraints
        A.resize(6,n+1);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n+2,n+1);
        d.resize(2*n+2);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i+n] = -1;
            d[i+n] = -tauMin;
        }
        // constraints on alpha
        d[2*n] = 1;
        C[2*n][2*n] = 1;
        C[2*n+1][2*n+1] = -1;
    }
    vpSubColVector tau(x, 0, n);

    vpColVector g(6), err, err_i(6), err0(6);
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6);

    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M;
    vpRotationMatrix R;

    // gain
    double Kp = 100, Ki = 0.5, Kd = 2;  // tuned for Caroca
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
                if(w[i] < robot.mass()*9.81)
                    err_i[i] += err[i] * dt;

            w = Kp * (err + Ki*err_i);

            // D term?
            if(err0.infinityNorm())
                w += Kp * Kd * (err - err0)/dt;
            err0 = err;
            // remove gravity
            w = w-g;
            cout << "Desired wrench in platform frame: " << (RR.transpose()*w).t() << fixed << endl;

            // build W matrix depending on current attach points
            robot.computeW(W);


            if(control == noMin)
                x = W.pseudoInverse() * RR.transpose()* w;
            else if(control == minT)
                solve_qp::solveQP(Q, r, W, w, C, d, x);
            else if(control == minW)
                solve_qp::solveQPi(W, w, C, d, x);
            else    // control = minA
            {
                for(int i=0;i<6;++i)
                {
                    for(int j=0;j<n;++j)
                        A[i][j] = W[i][j];
                    A[i][n+1] = -w[i];
                }
                solve_qp::solveQP(Q, r, A, b, C, d, x);
            }



            cout << "Checking W.tau+g in platform frame: " << (W*tau).t() << fixed << endl;
            cout << "sending tensions: " << tau.t() << endl;

            // send tensions
            robot.sendTensions(tau);
        }

        ros::spinOnce();
        loop.sleep();
    }





}

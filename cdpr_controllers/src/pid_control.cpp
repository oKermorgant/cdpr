
#include <ros/ros.h>
#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>
#include <log2plot/logger.h>
#include <chrono>
#include <cdpr_controllers/butterworth.h>

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
    robot.setDesiredPose(0,0,1,0,0,0);

    const bool force_cont = true;

    // log path
    std::string path = "/home/olivier/Results/cdpr/rel_";

    if(force_cont)
        path += "cont_";

    double tauMin, tauMax;
    robot.tensionMinMax(tauMin, tauMax);

    // QP variables
    vpColVector x(n);
    vpMatrix Q, A, C, W(6,n);
    vpColVector r, b, d, w(6);
    std::vector<bool> active;


    // get control type
    minType control = minA;
    /* std::string control_type;
    nh_priv.getParam("control", control_type);
    if(control_type == "noMin")
        control = noMin;
    else if(control_type == "minA")
        control = minA;
    else if(control_type == "minT")
        control = minT;
    else if(control_type == "minW")
        control = minW;
*/

    if(control == noMin)
        path += "noMin";
    else if(control == minT)
    {
        path += "minT";
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
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minW)
    {
        path += "minW";
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
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minA)
    {
        path += "minA";
        // min |tau| - alpha
        //  st W.tau = alpha.w
        //  st 0 < alpha < 1
        //  st t- < tau < t+
        x.resize(n+1);  // x = (tau, alpha)

        Q.eye(n+1);Q *= 1./tauMax;
        r.resize(n+1);r[n] = Q[n][n] = n*tauMax;
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
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
        // constraints on alpha
        d[2*n] = 1;
        C[2*n][n] = 1;
        C[2*n+1][n+1] = -1;
    }
    vpSubColVector tau(x, 0, n);

    vpColVector g(6), err, err_i(6), err0(6), v(6), v0(6), d_err(6);
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6);

    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M;
    vpRotationMatrix R;

    // gain
    double Kp = 200, Ki = 0.1, Kd = 3;  // tuned for Caroca
    Param(nh, "Kp", Kp);
    Param(nh, "Ki", Ki);
    Param(nh, "Kd", Kd);

    // variables to log
    log2plot::Logger logger(path + "_");
    double t;
    logger.setTime(t);
    vpPoseVector pose_err;
    //td::string name, const std::string legend, const std::string ylabel, const bool keep_file = true)
    logger.saveTimed(pose_err, "pose_err", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Pose error");
    logger.saveTimed(tau, "tau", "\\tau_", "Tensions");
    vpColVector residual(6);
    logger.saveTimed(residual, "res", "[f_x,f_y,f_z,m_x,m_y,m_z]", "Residuals");

    vpSubColVector alpha;
    if(control == minA)
    {
        alpha.init(x, n, 1);
        logger.saveTimed(alpha, "a", "[\\alpha]", "Alpha");
    }


    // chrono
    vpColVector comp_time(1);
    logger.saveTimed(comp_time, "dt", "[\\delta t]", "Comp. time");
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;

    // filter for d_error (dim. 6)
    Butterworth_nD filter(6, 1, dt);
    double dTau_max = 3;

    cout << "CDPR control ready" << fixed << endl;

    bool update_d = false;

    while(ros::ok())
    {
      //  cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        nh.getParam("Ki", Ki);
        nh.getParam("Kd", Kd);
        t = ros::Time::now().toSec();

        if(robot.ok())  // messages have been received
        {
            start = std::chrono::system_clock::now();
            // current position
            robot.getPose(M);
            M.extract(R);

            // position error in platform frame
            err = robot.getPoseError();
        //    cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];

            // position error in fixed frame
            err = RR * err;
          //  cout << "Position error in world frame: " << err.t() << fixed << endl;
            robot.sendError(err);
            // I term to wrench in fixed frame
            for(unsigned int i=0;i<6;++i)
                if(w[i] < robot.mass()*9.81)
                    err_i[i] += err[i] * dt;

            w = Kp * (err + Ki*err_i);

            // D term?
            if(err0.infinityNorm())
            {
                // compute and filter error derivative
                d_err =(err - err0)/dt;
                filter.Filter(d_err);

                w += Kp * Kd * d_err;
            }

            err0 = err;

       //     cout << "Desired wrench in fixed frame: " << w.t() << fixed << endl;
            // remove gravity + to platform frame
            w = RR.t()*(w-g);
        //   cout << "Desired wrench in platform frame: " << w.t() << fixed << endl;

            // build W matrix depending on current attach points
            robot.computeW(W);

            if(update_d && control != noMin)
            {
                for(unsigned int i=0;i<n;++i)
                {
                    d[i] = std::min(tauMax, tau[i]+dTau_max);
                    d[i+n] = -std::max(tauMin, tau[i]-dTau_max);
                }
                cout << "new d: " << d.t() << endl;

            }

            if(control == noMin)
                x = W.pseudoInverse() * w;
            else if(control == minT)
                solve_qp::solveQP(Q, r, W, w, C, d, x, active);
            else if(control == minW)
                solve_qp::solveQPi(W, w, C, d, x, active);
            else    // control = minA
            {
                for(int i=0;i<6;++i)
                {
                    for(int j=0;j<n;++j)
                        A[i][j] = W[i][j];
                    A[i][n] = -w[i];
                }
                solve_qp::solveQP(Q, r, A, b, C, d, x, active);
          //      cout << "alpha = " << alpha[0] << endl;
          //      cout << "checking W.tau - a.w: " << (W*tau - alpha[0]*w).t() << endl;
            }

         //   cout << "Residual: " << (W*tau - w).t() << fixed << endl;
        //    cout << "sending tensions: " << tau.t() << endl;

            // send tensions
            robot.sendTensions(tau);

            end = std::chrono::system_clock::now();
            elapsed_seconds = end-start;
            // log
            M.buildFrom( robot.getPoseError());
            pose_err.buildFrom(M.inverse());
            comp_time[0] = elapsed_seconds.count();
            residual = W*tau - w;
            logger.update();
            update_d = force_cont;
        }

        ros::spinOnce();
        loop.sleep();
    }

    logger.plot();
}

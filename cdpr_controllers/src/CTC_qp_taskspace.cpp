
#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>
#include <log2plot/logger.h>
#include <chrono>
#include <cdpr_controllers/butterworth.h>

using namespace std;


/*
 * Computed torque controller working in Cartesian space to show input/output of the CDPR class
 *
 * Does not consider the feasible wrench at the initial phase, namely probably not fulfill the equlity constraint 
 * Minimize the difference between the desired wrench and the computed wrench
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

    // log path
    std::string path = "/home/derek/Results/cdpr/";

    // initialization of parameters in CTC 
    vpMatrix W(6, n), RR(6,6);  
    vpColVector g(6), tau(n), err(6), T(n);
    g[2] = - robot.mass() * 9.81;

    // set frequency of loop
    double dt = 0.01;
    ros::Rate loop(1/dt);

    // declare the homogeneous matrix
    vpHomogeneousMatrix M, Md;
    vpRotationMatrix R,R_p,R_pp;

    // set proportional and derivative gain
    double Kp = 15, Kd = 15 ;  // tuned for Caroca

    Param(nh, "Kp", Kp);
    Param(nh, "Kd", Kd);
    
    // QP variables
    double fmin, fmax;   
    robot.tensionMinMax(fmin, fmax);
    vpMatrix C, M_inertia(6,6), Q;

    // declare desired parameter
    vpColVector a_d, v_d, v, v_e;
    v_d.resize(6);
    a_d.resize(6);
    v.resize(6);


    vpColVector b(6),r,d;
    r.resize(n);
    Q.resize(n,n);
    // constraints = fmin < f < fmax
    C.resize(2*n, n);
    d.resize(2*n);

    for(unsigned int i=0;i<n;++i)
        for(unsigned int k=0;k<n;++k)
           Q[i][k]=1;

    for(unsigned int i=0;i<n;++i)
    {
        // f < fmax
        C[i][i] = 1;
        d[i] = fmax;

        // -f < -fmin
        C[i+n][i] = -1;
        d[i+n] = -fmin;
    }

    M_inertia[0][0]=M_inertia[1][1]=M_inertia[2][2]=robot.mass();
    std::vector<bool> active;

     // variables to log
    log2plot::Logger logger(path);
    double t;
    logger.setTime(t);
    vpPoseVector pose_err;
    logger.saveTimed(pose_err, "pose_err", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Pose error");
    logger.saveTimed(tau, "tau", "\\tau_", "Tensions");

    // chrono
    vpColVector comp_time(1);
    logger.saveTimed(comp_time, "dt", "[\\delta t]", "Comp. time");
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;

    // filter for d_error (dim. 6)
    Butterworth_nD filter(6, 1, dt);
    vpPoseVector err_;

    cout << "CDPR control ready ----------------" << fixed << endl; 
    while(ros::ok())
    {
        //cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
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
            cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];
            // transform to reference frame
            err = RR* err;

            // get the desired parameters from trajectory generator
            robot.getVelocity(v);
            robot.getDesiredVelocity(v_d);
            robot.getDesiredAcceleration(a_d);

             //cout << "Desired acc: " << a_d.t() << fixed << endl;

             M_inertia.insert(robot.inertia(),3,3);
             // M_inertia.insert((R*robot.inertia()*R.t()),3,3); 
             v_e=v_d-v;
             filter.Filter(err);

              b= M_inertia*(a_d+Kp*err+Kd*v_e)-g;

            //cout << "Desired wrench in platform frame: " << (RR.transpose()*(b-g)).t() << fixed << endl;
            //v=v_d-v;
            robot.sendError(err);
    
            //cout << " Velocity error: " << (v_d-v).t()<< endl;
            //cout << " Inertia matrix: " <<M_inertia<< endl;
            //cout << " b: " << b.t()<< endl;
            // build W matrix depending on current attach points
            robot.computeW(W);
            
            // solve with QP
            // min ||QT-r||
            // st: C.T<=d  (fmin < f < fmax)
            // st: W.T=M.xdd-g
            //solve_qp::solveQP(Q,r,RR*W,b,C,d,T);
           
          
            // solve with QP
            // min ||W.T + g - tau||->||W.T-b||
            // st:st: C.T<=d  (fmin < f< fmax)
            solve_qp::solveQPi(W, RR.t()*b, C, d, tau, active);

            // Pseudoinverse method
            //T=W.pseudoInverse()*RR.transpose()*b;
            //cout << "Desired wrench in platform frame: " << (RR.transpose()*(tau - g)).t() << fixed << endl;

            //cout << "Checking W.f+g in platform frame: " << (W*T).t() << fixed << endl;

            cout << "sending tensions: " << tau.t() << endl;

            // send tensions
            robot.sendTensions(tau);

            // calculate the computation period
            end = std::chrono::system_clock::now();
            elapsed_seconds = end-start;
            // log
            for (int i = 0; i < 6; ++i)
                pose_err[i]=err[i];
            //pose_err=err;
            //M.buildFrom(robot.getPoseError(););
            //pose_err.buildFrom(M.inverse());
            comp_time[0] = elapsed_seconds.count();
            //residual = W*tau - w;
            logger.update();
        }

        ros::spinOnce();
        loop.sleep();
    }
     logger.plot();
}

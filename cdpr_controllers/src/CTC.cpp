
#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>
#include <log2plot/logger.h>
#include <chrono>
#include <cdpr_controllers/butterworth.h>
#include <cdpr_controllers/ctd.h>


using namespace std;


/*
 * Computed torque controller to show input/output of the CDPR class
 *
 *  minW does not consider the feasible wrench at the initial phase, namely probably not fulfill the equlity constraint 
 * Minimize the difference between the desired wrench and the computed wrench
 *
 * minT satisfies equality condition with feasible tensions
 *
 * minA new TDA 
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
    ros::NodeHandle nh, nh_priv("~");;

    // init CDPR class from parameter server
    CDPR robot(nh);
    const unsigned int n = robot.n_cables();

    // log path
    std::string path = "/home/derek/Results/cdpr/";

    // get control type    
    std::string control_type = "minW";
    double dTau_max = 0;
    bool warm_start = false;

    if(nh_priv.hasParam("control"))
    nh_priv.getParam("control", control_type);

    CTD::minType control = CTD::minA;

    if(control_type == "noMin")
        control = CTD::noMin;
    else if(control_type == "minT")
        control = CTD::minT;
    /*#############*/
    else if(control_type == "minTs")
        control = CTD::minTs;
    else if(control_type == "minAA")
        control = CTD::minAA;
 /*#################*/
    else if(control_type == "minW")
        control = CTD::minW;
    else if(control_type == "closed_form")
        control = CTD::closed_form;

    // get space type
    std::string space_type="Cartesian_space";
    if (nh_priv.hasParam("s_type"))
         nh_priv.getParam("s_type", space_type);
    
    // initialization of parameters in CTC 
    vpMatrix W(6, n), Wd(6,n), J(n,6), RR(6,6), RR_d(6,6),  M_inertia(6,6);;  
    vpColVector g(6), tau(n), err(6), T(n), w(6);
    vpColVector L(n), Ld(n), Le(n),  Le_d(n);
    g[2] = - robot.mass() * 9.81;
    double a;

    // set frequency of loop
    double dt = 0.01;
    ros::Rate loop(1/dt);

    // declare the homogeneous matrix
    vpHomogeneousMatrix M, Md;
    vpRotationMatrix R, Rd;

    // set proportional and derivative gain
    double Kp, Kd;  // tuned for Caroca
   if (space_type == "Cartesian_space")
        Kp=Kd=25;
    else if ( space_type == "Joint_space")
        Kp=Kd=1000;
    else 
        cout << "Gain error" << endl;

    Param(nh, "Kp", Kp);
    Param(nh, "Kd", Kd);
    
    // declare desired parameter
    vpColVector a_d, v_d, v, v_e;
    v_d.resize(6);
    a_d.resize(6);
    v.resize(6);

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
    Butterworth_nD filterP(6, 1, dt);
    Butterworth_nD filterL(8, 1, dt);
    vpPoseVector err_;

    // deliver the settings to the CTD
    CTD ctd(robot, control);
    ctd.ForceContinuity(dTau_max);

    cout << "CDPR control ready ----------------" << fixed << endl; 
    while(ros::ok())
    {
        //cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        nh.getParam("Kd", Kd);
        t = ros::Time::now().toSec();
        
        if(robot.ok())  // messages have been received
        {
            // extract the current time
             start = std::chrono::system_clock::now();
            // current poses
            robot.getPose(M);
            M.extract(R);
            // desired poses
            robot.getDesiredPose(Md);
            Md.extract(Rd);

            // position error in platform frame
            err = robot.getPoseError();
            //cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];
            // transform to reference frame
            err = RR* err;

            // create transformation matrix
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR_d[i][j] = RR_d[i+3][j+3] = Rd[i][j];

            // get the desired parameters from trajectory generator
            robot.getVelocity(v);
            robot.getDesiredVelocity(v_d);
            robot.getDesiredAcceleration(a_d);

             //cout << "Desired acc: " << a_d.t() << fixed << endl;

             //M_inertia.insert(robot.inertia(),3,3);
             M_inertia.insert((R*robot.inertia()*R.t()),3,3); 
            // build W matrix depending on current attach points
             robot.computeW(W);
             W=RR*W;

             if ( space_type == "Cartesian_space")
             {             
                 v_e=v_d-v;
                 //filterP.Filter(err);

                // establish the external wrench 
                w= M_inertia*(a_d+Kp*err+Kd*v_e)-g;
                //v=v_d-v;
                //robot.sendError(err);
                cout << "controller in task space" << endl;
        
                //cout << " Velocity error: " << (v_d-v).t()<< endl;
                //cout << " Inertia matrix: " <<M_inertia<< endl;
                //cout << " b: " << b.t()<< endl;
                
            }
            else if ( space_type == "Joint_space")
            {
                J=  -W.t();
                // computation of cables length
                robot.computeLength(L);
                robot.computeDesiredLength(Ld);

                w = M_inertia*a_d - g;

                //  the desired structure matrix
                robot.computeDesiredW(Wd);
                // transform the structure matrix to platform space
                Wd=RR_d*Wd;
                robot.sendError(err);

                Le_d=  -Wd.t() *v_d- J*v;
                Le= Ld-L;
                //filterL.Filter(Le);
                //robot.sendLengthError(Le);
                cout << "length error:" << Le.t() << endl;

                tau=Kp*Le+Kd*Le_d;
                w += - Wd* tau;
                //cout << " b: " << b.t()<< endl;
                cout << "controller in Joint space" << endl;
            }
            else
                cout << " Error: Please select the controller space type" << endl;

            // call cable tension distribution
            tau = ctd.ComputeDistribution(W, w)  ;
          
            // send tensions
            robot.sendTensions(tau);
            ctd.GetAlpha(a);
            cout << "coefficient number:" << a << endl;

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


#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>
#include <log2plot/logger.h>
#include <chrono>
#include <cdpr_controllers/butterworth.h>
#include <cdpr_controllers/tda.h>
#include <visp/vpIoTools.h>

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
    std::string path = "/home/" + vpIoTools::getUserName() + "/Results/cdpr/";

    // get control type    
    std::string control_type = "Barycenter";
    double dTau_max = 0.5, lambda =5000;
    bool warm_start = false;

    if(nh_priv.hasParam("control"))
        nh_priv.getParam("control", control_type);
    if(nh_priv.hasParam("threshold"))
        nh_priv.getParam("threshold", dTau_max);
    if(nh_priv.hasParam("coefficient"))
        nh_priv.getParam("coefficient", lambda);


    TDA::minType control = TDA::minA;

    if(control_type == "noMin")
        control = TDA::noMin;
    else if(control_type == "minT")
        control = TDA::minT;
    else if(control_type == "minW")
        control = TDA::minW;
    else if(control_type == "closed_form")
        control = TDA::closed_form;
    else if(control_type == "Barycenter")
        control = TDA::Barycenter;
    else if(control_type == "minG")
        control = TDA::minG;
     else if(control_type == "CGAL")
        control = TDA::cgal;
    
    // get space type
    std::string space_type="Cartesian_space";
    if (nh_priv.hasParam("s_type"))
         nh_priv.getParam("s_type", space_type);
    
    // initialization of parameters in CTC 
    vpMatrix W(6, n), Wd(6,n), J(n,6), RR(6,6), RR_d(6,6),  M_inertia(6,6), Kp(6,6), Kd(6,6);
    vpColVector g(6), tau(n), err(6),  w(6), tau0(n), tau_diff(n), pd(6),residual(6);
    vpColVector L(n), Ld(n), Le(n),  Le_d(n);
    g[2] = - robot.mass() * 9.81;
    //vpPoseVector Pd;
    vpRxyzVector rxyz;
    // set frequency of loop
    double dt = 0.01;
    ros::Rate loop(1/dt);

    // declare the homogeneous matrix
    vpHomogeneousMatrix M, Md;
    vpRotationMatrix R, Rd;
    vpTranslationVector T;

    // set proportional and derivative gain
    //double Kp, Kd;  // tuned for Caroca
   if (space_type == "Cartesian_space")
     {  
        for (int i = 0; i < 3; ++i)
                {
                    Kp[i][i] = 100; Kd[i][i] = 20;
                    Kp[i+3][i+3]= 100; Kd[i+3][i+3]= 20;
                }
    }
    else if ( space_type == "Joint_space")
        Kp=Kd=1000;
    else 
        cout << "Gain error" << endl;

    //Param(nh, "Kp", Kp);
    //Param(nh, "Kd", Kd);
    
    // declare desired parameter
    vpColVector a_d, v_d, v, v_e;
    v_d.resize(6);
    a_d.resize(6);
    v.resize(6);
    v_e.resize(6);

    M_inertia[0][0]=M_inertia[1][1]=M_inertia[2][2]=robot.mass();
    std::vector<bool> active;

     // variables to log
    log2plot::Logger logger(path);
    double t;
    logger.setTime(t);
    vpPoseVector pose_err;
    vpThetaUVector orientation_err;
    vpTranslationVector position_err;
    // logger.saveTimed(pose_err, "pose_err", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Pose error");
    logger.saveTimed(orientation_err, "orientation_err", "[\\theta_x,\\theta_y, \\theta_z]", "Orientation error [deg]" );
    logger.saveTimed(position_err, "position_err", "[x, y, z]", "Position error [m]");
    logger.saveTimed(tau, "tau", "\\tau_", "Tensions [N]");
    logger.saveTimed(residual, "residual", "residual_", "wrench residual [N]");
    logger.saveTimed(tau_diff, "diff", "\\tau_d", "Tensions difference [N]");
    logger.saveTimed(v_e, "velocity_error", "Vel_", "velocity error [N]");
    if (space_type == "Joint_space" )
        logger.saveTimed(Le, "Le", "Le_", "Length error [m]");

    // chrono
    vpColVector comp_time(1), alpha(1), sum(1),ver(1), gains(2);
    logger.saveTimed(comp_time, "dt", "[\\delta t]", "Comp. time [s]");
    if (control_type == "minA")
        logger.saveTimed(alpha, "alpha", "[\\alpha ]", "Alpha");
    if (control_type == "Barycenter")
        logger.saveTimed(ver, "vertices", "[numv]", "The number of vertex");
    if (control_type == "minG")
        logger.saveTimed(gains, "gains", "[K_p, K_d]", "CTC gains");
    
    // initialize the timekeeper 
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;

    // filter for d_error (dim. 6)
    Butterworth_nD filterP(6, 1, dt);
    Butterworth_nD filterL(8, 1, dt);
    vpPoseVector err_;

    // deliver the settings to the TDA
    TDA tda(robot, nh, control);
    tda.ForceContinuity(dTau_max);
    tda.Weighing(lambda);

    cout << "CDPR control ready ----------------" << fixed << endl; 
    while(ros::ok())
    {
        //cout << "------------------" << endl;
        //nh.getParam("Kp", Kp);
        //nh.getParam("Kd", Kd);
        t = ros::Time::now().toSec();
          robot.getPose(M);
            M.extract(T);

        if(robot.ok())  // messages have been received
        {
            cout << "messages have been received" << endl;

            // current poses
            robot.getPose(M);
            M.extract(R);

            // desired poses
            robot.getDesiredPose(Md);
            Md.extract(Rd);
            pd=vpPoseVector(Md);
            vpQuaternionVector Qd,Q;
            vpThetaUVector Theta_c;
            Md.extract(Qd);
            M.extract(Theta_c);
            cout<<" the  angle-axis angle"<< "  "<< Theta_c.t()<< endl;
            //cout<<"the desired quaternion angle"<< "  "<< Qd.t()<< endl;
            M.extract(Q);
            cout<<" the  quaternion angle"<< "  "<< Q.t()<< endl;


            // get the desired parameters from trajectory generator
            robot.getVelocity(v);
            robot.getDesiredVelocity(v_d);
            robot.getDesiredAcceleration(a_d);
            rxyz.buildFrom(R);

            // position error in platform frame
            err = robot.getPoseError();
            //cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];
            // transform to reference frame
            err=RR*err;
            rxyz= -1*rxyz;
            err.insert(3, rxyz);
            cout << " Pose error:" <<"  "<<err.t() << endl;

            // create transformation matrix
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR_d[i][j] = RR_d[i+3][j+3] = Rd[i][j];


            cout << " Current position:" <<"  "<<T.t() << endl;
            cout << " Current velocity: " << "  "<<v.t()<< endl;

             // transform the inertia matrix to reference frame
             M_inertia.insert((R*robot.inertia()*R.t()),3,3);
             cout << " Inertia matrix: " << "  \n"<<R*robot.inertia()*R.t()<< endl;
             // build W matrix depending on current attach points
             robot.computeW(W);
             W=RR*W;

             if ( space_type == "Cartesian_space")
             {             
                // compute the velocity error
                 v_e= v_d - v;
                 // add Butterworth filter for pose error
                 filterP.Filter(err);

                 if ( control_type == "minG") 
                        w = M_inertia*a_d-g;            
                else
                    // establish the external wrench 
                    w = M_inertia*(a_d+Kp*err+Kd*v_e)-g;  
                cout << "controller in task space" << endl;              
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
                //robot.sendError(err);

                Le_d=  - Wd.t() *v_d- J*v;
                Le= Ld-L;
                filterL.Filter(Le);

                cout << "length error:" << Le.t() << endl;

                tau=Kp*Le+Kd*Le_d;
                w += - Wd* tau;
                //cout << " b: " << b.t()<< endl;
                cout << "controller in Joint space" << endl;
            }
            else
                cout << " Error: Please select the controller space type" << endl;

             tau0=tau;
             
             // extract the current time
             start = std::chrono::system_clock::now();
             // call cable tension distribution
             if ( control_type == "minG")
             {
                v_e = M_inertia*v_e;
                err = M_inertia*err;
                tau = tda.ComputeDistributionG(W, v_e, err, w) ;
            }
            else
                tau = tda.ComputeDistribution(W, w) ;

             end = std::chrono::system_clock::now();

             cout << "external wrench:" << "   "<< w.t() << endl;

             // compute the tension difference
             tau_diff= tau-tau0;

            // send tensions
            robot.sendTensions(tau);

            if(control_type == "minA")
                {
                    tda.GetAlpha(alpha[0]);
                    cout << "coefficient number:" << "  " <<alpha << endl;
                }
            if(control_type == "Barycenter")
                    tda.GetVertices(ver[0]);
            if(control_type == "minG")
                    tda.GetGains(gains);

            // calculate the computation period
            elapsed_seconds = end-start;
            // log
            M.buildFrom( robot.getPoseError());
            pose_err.buildFrom(M.inverse());
            for (int i = 0; i < 3 ; ++i)
             {   
                position_err[i]=pose_err[i];
                orientation_err[i]=(pose_err[i+3]*(180/M_PI));
            }
            // computation time
            comp_time[0] = elapsed_seconds.count();
            residual = W*tau - w;
            logger.update();
        }

        ros::spinOnce();
        loop.sleep();
    }
     logger.plot();
}

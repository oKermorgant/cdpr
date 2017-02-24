#include <cdpr_controller/cdpr4qp.h>
#include <cdpr_controller/qp_slover.h>

using namespace std;
//using namespace gazebo;


/*
 * Feedback linearization controller to show input/output of the CDPR class
 *
 * Combine with the qp slover to get optimal force
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

    // precision 4 on 1.23456 you get 1.234 
    cout.precision(3);
    // init ROS node
    ros::init(argc, argv, "cdpr_qp_control");
    ros::NodeHandle nh;


    // load model parameters  ??????
    //ros::NodeHandle model(nh, "model");
     CDPR robot(nh);
    const unsigned int n = robot.n_cables();

    
    vpMatrix W(6, n);   // tau = W.T + g
    vpColVector g(6), tau(6), err, err_p, err_a,T(n), err_d(6),b(6),r,d;
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6), RR_p(6,6);
    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M, Md, Md_p;
    vpRotationMatrix R, R_p;
    vpQuaternionVector q;
    vpTranslationVector t_d,t;
    vpMatrix C, M_inertia(6,6),Q;
    double fmin,fmax;
    
    // initialize the qp_slover matrix
    // minimize matrix
    r.resize(n);
    d.resize(2*n);
    Q.resize(n,n);
    C.resize(2*n,n);

    // objective function matrix
    for(unsigned int i=0;i<n;++i)
        for(unsigned int k=0;k<n;++k)
           Q[i][k]=1;

    // inequality matrix
    for(unsigned int i=0;i<n;++i)
        for(unsigned int k=0;k<n;++k)
        {
           C[i][k]=1;
           C[i+n][k]=-1;
        }

    robot.tensionMinMax(fmin,fmax);
    for(unsigned int i=0;i<n;++i)
        {
            d[i]=fmax;
            d[i+n]=-fmin;
        }

    // gain
    double Kp = 50, Kd = 50;  // Ki = 0.5, tuned for Caroca
    Param(nh, "Kp", Kp);
    // Param(nh, "Ki", Ki);
    Param(nh, "Kd", Kd);
    
    // previous desired matrix
    robot.getInitialPose(Md_p);
    cout << "CDPR control ready" << fixed << endl;
    while(ros::ok())
    {
        cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        //nh.getParam("Ki", Ki);
         nh.getParam("Kd", Kd);

        
         
        if(robot.ok())   // messages have been received
        {


            // current position
            robot.getPose(M);
            M.extract(R);
            M.extract(t);

            // deisired position ??????
            robot.getDesiredPose(Md);
           
            // position error in platform frame
            err = robot.getPoseError();
            cout << "Position error in platform frame: " << err.t() << fixed << endl;
            // RR is one 6x6 matrix
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];

            // calculate the desired acceleration
            err_p= robot.getDesiredPoseError(Md_p,Md);
            Md_p.extract(R_p);
            Md_p=Md;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR_p[i][j] = RR_p[i+3][j+3] = R_p[i][j];


            // position error in fixed frame
            err = RR * err;
            // desired pose error between previous and current position in fixed frame
            err_p= RR_p * err_p;

            for(unsigned int i=0;i<6;++i)
                // the desired acceleration
                err_a[i] = err_p[i] / (dt*dt);

            // I term to wrench in fixed frame
            for(unsigned int i=0;i<6;++i)
                //if(tau[i] < robot.mass()*9.81)
                    //calculate the difference of velocity
                    err_d[i] = err[i] / dt;
            M_inertia=robot.inertia();

            // create the qp matrix equality constraint
            b=-(M_inertia*(err_a+Kp*err+Kd*err_d)+g);
            // build W matrix depending on current attach points
            robot.computeW(W);

            //solve_qp::solveQP(Q,r,W,b,C,d,T);
            //T=W.pseudoInverse()*b;
            
            // Closed form method
            // f_m= (f_max+f_min)/2;
            // T= f_m-W.pseudoInverse()*(-b + (W*f_m));





	        //cout << "Desired force in platform frame: " << T.t() << fixed << endl;
            cout << " The tesion in cables: " << T.t() << endl;
                
            //tau = Kp * (err + Ki*err_i);
            // position error in fixed frame
            //tau = fRRp * tau;
            // PI controller to wrench in fixed frame
           /* tau_i += tau * dt;
            tau = Kp * (tau + Ki*tau_i);
            cout << "Desired wrench in platform frame: " << (fRRp.transpose()*(tau - g)).t() << fixed << endl;
           */
            // solve problem : tau = T.u + g
            //qp_r = fRRp.transpose() * (tau-g);
            //qp.solveCascade(u);
            //u = T.pseudoInverse() * fRRp.transpose()* (tau - g);
            //cout << "   Verif W.f+g in platform frame: " << (T*u).t() << fixed << endl;
            //u = 0.5*(u+u0);
            //u0 = u;

           // write effort to jointstate
           cout << "sending tensions: " << T.t() << endl;
           // send tensions
           robot.sendTensions(T);

        }

        ros::spinOnce();
        loop.sleep();
    }


}

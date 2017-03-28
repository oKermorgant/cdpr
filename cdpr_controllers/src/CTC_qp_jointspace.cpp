
#include <cdpr/cdpr.h>
#include <cdpr_controllers/qp.h>

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

    vpMatrix W(6, n), J(n,6);   // tau = W.T+ g
    vpColVector g(6), err(6), T(n), err_d(6), err0(6),err_p(6), err_a(6),err_i(6);
    vpColVector L(n), tau(n), Le_d(n), Ld(n), Le(n), L_p(n), L_d(n), Ld_p(n), Ld_d(n);
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6),RR_d(6,6), Wd(6,n);

    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M, Md, Md_p,Md_pp;
    vpRotationMatrix R,Rd;

    // gain
    double Kp = 1000, Kd = 1000 ;  // tuned for Caroca

    Param(nh, "Kp", Kp);
    Param(nh, "Kd", Kd);

    // QP variables
    double fmin, fmax;    robot.tensionMinMax(fmin, fmax);
    vpMatrix C,M_inertia(6,6),Q;

    // definition of the closed form parameter
     vpColVector f_v, f_mM;
     double f_m;
     f_m= (fmax+fmin)/2;
     f_v.resize(n);
     f_mM.resize(n);

    for (unsigned int i = 0; i < robot.n_cables(); ++i)
            {
               f_mM[i]=f_m;
            }

    // desired parameter for qp solver
    vpColVector a_d, v_d, v;
    v_d.resize(6);
    a_d.resize(6);
    v.resize(6);

    // initialize the constraint condition for the qudratic  programming 
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

    cout << "CDPR control ready" << fixed << endl; 
    while(ros::ok())
    {
        cout << "------------------" << endl;
        nh.getParam("Kp", Kp);
        //nh.getParam("Ki", Ki);
        nh.getParam("Kd", Kd);

        if(robot.ok())  // messages have been received
        {
            // current position
            robot.getPose(M);
            M.extract(R);
             robot.getDesiredPose(Md);
             Md.extract(Rd);
            // position error in platform frame
            err = robot.getPoseError();
            cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];

            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR_d[i][j] = RR_d[i+3][j+3] = Rd[i][j];

            err = RR * err;

            // obtain the velocity and the acceleration
            robot.getVelocity(v);
            robot.getDesiredVelocity(v_d);
            robot.getDesiredAcceleration(a_d);

             cout << "Desired acc: " << a_d.t() << fixed << endl;

             M_inertia.insert(robot.inertia(),3,3);
             //M_inertia.insert((R*robot.inertia()*R.t()),3,3); 

            // equality  constraint 
            b=M_inertia*a_d-g;
            //cout << "Desired wrench in platform frame: " << (RR.transpose()*(b-g)).t() << fixed << endl;
          
            
            // build W matrix depending on current attach points
            robot.computeW(W);

            W= RR*W;
            J=  -W.t();
            // computation of cables length
            robot.computeLength(L);
            robot.computeDesiredLength(Ld);

            //  the desired structure matrix
            robot.computeDesiredW(Wd);

            Wd=RR_d*Wd;

            robot.sendError(err);
     

            Le_d=  -Wd.t() *v_d- J*v;

            Le= Ld-L;
            robot.sendLengthError(Le);
            cout << "length error:" << Le.t() << endl;

           
             tau=Kp*Le+Kd*Le_d;
             b += - Wd* tau;
             cout << " b: " << b.t()<< endl;



            // solve with QP
            // min ||QT-r||
            // st: C.T<=d  (fmin < f < fmax)
            // st: W.T=M.xdd-g
            //solve_qp::solveQP(Q,r,RR*W,b,C,d,T);
           
          
            // solve with QP
            // min ||W.T + g - tau||->||W.T-b||
            // st:st: C.T<=d  (fmin < f< fmax)
            //solve_qp::solveQPi(W, b, C, d, T);

            // Pseudoinverse method
            //T=W.pseudoInverse()*RR.transpose()*b;
            // T=W.pseudoInverse()*b;
            //cout << "Desired wrench in platform frame: " << (RR.transpose()*(tau - g)).t() << fixed << endl;

              // construct the equality constriant
            f_v= W.pseudoInverse()*(b - (W*f_mM));  

            // solve with QP
            // min ||Q.f_v -F_v||
            // st fmin < f_v < fmax
            //solve_qp::solveQP(Q,r,W,b,C,d,T);

            solve_qp::solveQPi(Q, f_v, C, d, T);

             T=f_mM+T;
             
            cout << "The wrench implemented by the cables: " << (W*T).t() << fixed << endl;
            cout << "Desired external wrench in platform frame: " << (W*T+g).t()<< fixed << endl;
            cout << "sending tensions: " << T.t() << endl;

            // send tensions
            robot.sendTensions(T);
        }

        ros::spinOnce();
        loop.sleep();
    }

}

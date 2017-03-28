
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

    vpMatrix W(6, n);   // tau = W.T+ g
    vpColVector g(6), tau(6), err(6), T,  err0(6);
    g[2] = - robot.mass() * 9.81;
    vpMatrix RR(6,6);

    
    double dt = 0.01;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M, Md, Md_p,Md_pp;
    vpRotationMatrix R,R_p,R_pp;

    // gain
    double Kp = 15, Kd = 15 ;  // tuned for Caroca

    Param(nh, "Kp", Kp);
    Param(nh, "Kd", Kd);
    
    // QP variables
    double fmin, fmax;    robot.tensionMinMax(fmin, fmax);
    vpMatrix C,M_inertia(6,6),Q;

    // desired parameter
    vpColVector a_d, v_d, v;
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

            // position error in platform frame
            err = robot.getPoseError();
            cout << "Position error in platform frame: " << err.t() << fixed << endl;
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    RR[i][j] = RR[i+3][j+3] = R[i][j];


            //err = RR* err;
            robot.getVelocity(v);
            robot.getDesiredVelocity(v_d);
            robot.getDesiredAcceleration(a_d);

             cout << "Desired acc: " << a_d.t() << fixed << endl;

             M_inertia.insert(robot.inertia(),3,3);
             // M_inertia.insert((R*robot.inertia()*R.t()),3,3); 

              b= M_inertia*(a_d+Kp*err+Kd*(v_d-v))-g;

            //cout << "Desired wrench in platform frame: " << (RR.transpose()*(b-g)).t() << fixed << endl;
            err=Kp*err;
            //v=v_d-v;
             v=(err - err0)/dt;
            robot.sendError(err);
    
            cout << " Velocity error: " << (v_d-v).t()<< endl;
            cout << " Inertia matrix: " <<M_inertia<< endl;
            cout << " b: " << b.t()<< endl;
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
            solve_qp::solveQPi(W, RR.t()*b, C, d, T);

            // Pseudoinverse method
            //T=W.pseudoInverse()*RR.transpose()*b;
            //cout << "Desired wrench in platform frame: " << (RR.transpose()*(tau - g)).t() << fixed << endl;

            //cout << "Checking W.f+g in platform frame: " << (W*T).t() << fixed << endl;
            cout << "sending tensions: " << T.t() << endl;

            // send tensions
            robot.sendTensions(T);
        }

        ros::spinOnce();
        loop.sleep();
    }

}

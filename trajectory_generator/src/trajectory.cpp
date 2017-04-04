
#include <trajectory_generator/trajectory.h>
#include <log2plot/logger.h>\

/*--------------------------------------------------------------------
*  5 order polynomial motion planning algorithm
*  This is the trajectory generator to generate continuous curve
*--------------------------------------------------------------------*/


using namespace std;
using namespace log2plot;

int main(int argc, char ** argv)
{
        cout.precision(3);
        // init ROS node
        ros::init(argc, argv, "trajectory_generator");
        ros::NodeHandle node;

        Trajectory path(node);
        std::string dir = "/home/derek/Results/cdpr/";;
        // plotting 3D figures
        vpPoseVector pose;
        Logger logger(dir);
        // save pose as 3D plot
        // the saved variable is the world pose in camera frame, we want to plot the invert
        logger.save3Dpose(pose, "trajectory", "box pose", false);


        vpRowVector x_i(3), x_f(3), v_i(3), v_f(3), a_i(3), a_f(3);
        vpRowVector P, Vel, Acc;
        double t_i,t_f, t_a, t_c,t;
        vpMatrix L, A, C;
        L.resize(6,6);
        A.resize(6,3);
        C.resize(6,3);

        path.InitializeTime(t_i,t_f);
        path.InitializePose(x_i,x_f);

        for (int i = 0; i < 3; ++i)
        {
          C[0][i]=x_i[i];
          C[3][i]=x_f[i];
        }
         L=path.getLmatrix(t_f);

        //A= L.pseudoInverse()*C;
        A= L.inverseByLU()*C;


        double dt = 0.01;
        ros::Rate loop(1/dt);
        int num=0, inter=0;
        num= t_f/dt;
        cout << "trajectory------------------" <<fixed << endl;
        while (ros::ok())
        {
              //cout << "trajectory------------------" << endl;
              //cout << " The condition matrix" << C<<endl;
              //cout << " The time matrix" << L<<endl;
              //cout << " The cofficient matrix" << A<<endl;

              // relative time from the beginning
              t=t_i+inter*dt;
              
              // Check the time period 
              if (inter<=(num+1))
              {
                P=path.getposition(t,A);
                Vel=path.getvelocity(t,A);
                Acc=path.getacceleration(t,A);
                path.sendDesiredpara(P.t(), Vel.t(), Acc.t());
              }
             else
               path.sendDesiredpara(P.t(), Vel.t(), Acc.t());

             // construct the pose vector
              pose.buildFrom(P[0], P[1],  P[2],  P[3],  P[4],  P[5]);

              // log
              logger.update();

                 //cout << " Desired position" << P <<endl;
                 // cout << " The trajectory has been tracked" << endl;
                 //cout << " Desired velocity" << Vel <<endl;
                 //cout << " Desired acceleration" << Acc <<endl;
              inter++;
              cout << " interation number" << inter <<endl;
              ros::spinOnce();
              loop.sleep();
        }
         logger.plot("", true);
         return 0;
};

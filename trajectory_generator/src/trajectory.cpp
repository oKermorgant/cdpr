#include <trajectory_generator/trajectory.h>
/*--------------------------------------------------------------------
*  5 order polynomial motion planning algorithm
*  This is the trajectory generator to generate continuous curve
*--------------------------------------------------------------------*/
using namespace std;

int main(int argc, char ** argv)
{
  cout.precision(3);
  // init ROS node
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle node;

  Trajectory tra(node);

  vpRowVector x_i(3), x_f(3), v_i(3), v_f(3), a_i(3), a_f(3);
  vpRowVector P, Vel, Acc;
  double t_i,t_f, t_a, t_c,t;
  vpMatrix L, A, C;
  L.resize(6,6);
  A.resize(6,3);
  C.resize(6,3);
  //tra.InitializeTime(t_i,t_f);
  t_i=0; t_f=5;
  //tra.InitializePose(x_i,x_f);

  x_i[0]=0; x_i[1]=0; x_i[2]= 1.5;
  x_f[0]=2; x_f[1]=2; x_f[2]= 1;
  for (int i = 0; i < 3; ++i)
  {
    C[0][i]=x_i[i];
    C[3][i]=x_f[i];
  }
   L=tra.getLmatrix(t_f);

  A= L.inverseByLU()*C;


  double dt = 0.01;
  ros::Rate loop(1/dt);
  int num=0, inter=0;
  num= t_f/dt;


  while (ros::ok())
  {
    cout << "trajectory------------------" << endl;

    // cout << " The condition matrix" << C<<endl;
    // cout << " The time matrix" << L<<endl;
    //cout << " The cofficient matrix" << A<<endl;

    // relative time from the beginning
    t=t_i+inter*dt;
    
    // Check the time period 
    if (inter<=num)
    {
      P=tra.getposition(t,A);
      Vel=tra.getvelocity(t,A);
      Acc=tra.getacceleration(t,A);
      tra.sendDesiredpara(P.t(), Vel.t(), Acc.t());
    }
   else
   {
     tra.sendDesiredpara(P.t(), Vel.t(), Acc.t());
   }


   cout << " Desired velocity" << P <<endl;
   // cout << " The trajectory has been tracked" << endl;
   // cout << " Desired velocity" << Vel <<endl;
    //cout << " Desired acceleration" << Acc <<endl;
    inter++;
    cout << " interation number" << inter <<endl;
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
};


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Pose.hh>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpMath.h>
#include <std_msgs/Float64MultiArray.h>
#include <cdpr3/trajectory.h>

using namespace std;
using namespace gazebo;

void Param(ros::NodeHandle &nh, const string &key, double &val)
{
    if(nh.hasParam(key))
        nh.getParam(key, val);
    else
        nh.setParam(key, val);
}

class Listener
{
public:
    Listener(ros::NodeHandle &_nh)
    {
        // init listener to gazebo link states
        gz_sub_ = _nh.subscribe("/gazebo/link_states", 1, &Listener::LinkStates_cb, this);
        //gz_received_ = false;

        // init listener to pose setpoint
        setpoint_sub_ = _nh.subscribe("pf_setpoint", 1, &Listener::Setpoint_cb, this);
        setpoint_received_ = false;



    }

    // callback for link states
    void LinkStates_cb(const gazebo_msgs::LinkStatesConstPtr &_msg)
    {
        
        vpHomogeneousMatrix M;
        for(unsigned int i=0;i<_msg->name.size();++i)
        {
            if(_msg->name[i] == "cube::platform")
            {
                t.set(_msg->pose[i].position.x, _msg->pose[i].position.y, _msg->pose[i].position.z); 
               M_.insert(vpTranslationVector(_msg->pose[i].position.x, _msg->pose[i].position.y, _msg->pose[i].position.z));
               M_.insert(vpQuaternionVector(_msg->pose[i].orientation.x, _msg->pose[i].orientation.y, _msg->pose[i].orientation.z,_msg->pose[i].orientation.w));
 gz_received_ = true;
            }
            else if(_msg->name[i] == "cube::frame")
            {
                               
                M.insert(vpTranslationVector(_msg->pose[i].position.x, _msg->pose[i].position.y, _msg->pose[i].position.z));
                M.insert(vpQuaternionVector(_msg->pose[i].orientation.x, _msg->pose[i].orientation.y, _msg->pose[i].orientation.z,_msg->pose[i].orientation.w));
            }
        }
        M_ = M_.inverse() * M;
 
 }


    // callback for pose setpoint
    void Setpoint_cb(const geometry_msgs::PoseConstPtr &_msg)
    {
        setpoint_received_ = true;
        Md_.insert(vpTranslationVector(_msg->position.x, _msg->position.y, _msg->position.z));
        Md_.insert(vpQuaternionVector(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z,_msg->orientation.w));
cout << "Recieved setpoint from Trajectory planner" << endl;
 
    }


    bool setpoint_received_, gz_received_;
    vpHomogeneousMatrix M_, Md_;
     vpTranslationVector t;

protected:
    ros::Subscriber gz_sub_, setpoint_sub_;

};
int iter = 0;
bool message_recieved = true;
vector<double> biggest;
// Actual end effector position
vector<double> positionx;
vector<double> positiony;
vector<double> positionz;

// Actual end effector velocity
vector<double> velocityx;
vector<double> velocityy;
vector<double> velocityz;

//Force profile during trajectory
vector<vector<double> > forces;

///Desired  end-effector trajectory
vector<double> position_x, position_y, position_z; 
vector<double> velocity_x, velocity_y, velocity_z;
vector<double> acceleration_x, acceleration_y, acceleration_z;  


ros::Subscriber force_sub;
ros::Subscriber force_sub2;

void Forcecb(const std_msgs::Float64MultiArray::ConstPtr& array)
{

//std::vector<double> myvec (array->data, array->data + sizeof(array->data) / sizeof(double) );
forces.push_back(array->data);

//myvec.clear();


}

void Platformcb(const nav_msgs::Odometry::ConstPtr& _msg)
{

positionx.push_back(_msg->pose.pose.position.x);
positiony.push_back(_msg->pose.pose.position.y);
positionz.push_back(_msg->pose.pose.position.z);

cout << "The actual positions are  . . . xa, ya , za" << "{  "<< _msg->pose.pose.position.x << "  "<< _msg->pose.pose.position.y << "  "<<_msg->pose.pose.position.z << " }"<<endl;

velocityx.push_back(_msg->twist.twist.linear.x);
velocityy.push_back(_msg->twist.twist.linear.y );
velocityz.push_back(_msg->twist.twist.linear.z);

//cout << "The actual positions are  . . . xa, ya , za" << "{  "<< _msg.twist.twist.position.x << "  "<< _msg.twist.twist.position.y << "  "<<_msg.twist.twist.position.z << " }"<<endl;
}



int main(int argc, char ** argv)
{

    cout.precision(3);

    // init ROS node
    ros::init(argc, argv, "cdpr_control");
    ros::NodeHandle nh;


    // load model parameters
    ros::NodeHandle model(nh, "model");

    // platform mass
    double mass;
    model.getParam("platform/mass", mass);
    cout << "This is the Mass that we found" << mass << endl;

   //Subscriber to the force message
   
   force_sub = nh.subscribe("applied_force_record", 1, &Forcecb);
   force_sub2 = nh.subscribe("cdpr_pose", 1, &Platformcb);
   //force_sub3 = nh.subscribe("cdpr_twist", 1, &Platformvb);
  

    // platform initial pose
    vector<double> xyz, rpy;
    model.getParam("platform/position/xyz", xyz);
    model.getParam("platform/position/rpy", rpy);
    vpHomogeneousMatrix M, Md;
    vpRxyzVector r(rpy[0], rpy[1], rpy[2]);
    Md.insert(vpRotationMatrix(r));
    Md.insert(vpTranslationVector(xyz[0], xyz[1], xyz[2]));

    // cable min / max
    double u_min, u_max;
    model.getParam("joints/actuated/effort", u_max);
    model.getParam("joints/actuated/min", u_min);
 
    Listener listener(nh);  
   vpTranslationVector t_(listener.t);


 cout << "Waiting to recieve the position of the Platform for Trajectory planning......" << endl; 
 
    // cable attach points
    XmlRpc::XmlRpcValue points;
    model.getParam("points", points);
    const unsigned int n = points.size();
    cout << "no of cables is" << n << endl;
    
    vector<vpTranslationVector> frame_p, platform_p;
    double x, y, z;
    for(unsigned int i=0;i<n;++i)
    {
        x = points[i]["frame"][0];
        y = points[i]["frame"][1];
        z = points[i]["frame"][2];
        frame_p.push_back(vpTranslationVector(x, y, z));
     

    }
     

    // publisher to cable tensions
    ros::Publisher cable_pub = nh.advertise<sensor_msgs::JointState>("cable_command", 1);
    ros::Publisher error_pub = nh.advertise< trajectory_msgs::JointTrajectoryPoint >("trajectory", 1000);
    sensor_msgs::JointState cable_sp;
    trajectory_msgs::JointTrajectoryPoint error_sp;
     error_sp.positions.resize(3, 0.0);
    error_sp.velocities.resize(3, 0.0);
    error_sp.accelerations.resize(3, 0.0);
    char cable_name[256];
    for(unsigned int i=0;i<n;++i)
    {
        sprintf(cable_name, "linear%i", i);
        cable_sp.name.push_back(string(cable_name));
    }
    cable_sp.effort.resize(n);

    // gain
    double Kp = 0.1, Ki = 0.01;
    Param(nh, "Kp", Kp);
    Param(nh, "Ki", Ki);
     
    //Trajectory    
    Trajectory trajx, trajy, trajz;
    cout << "t[0] iss " << t_[0] << endl;
    cout << "t[1] is " << t_[1] << endl;
    cout << "t[2] is " << t_[2] << endl;
   // vector<double> x_traj_coefficients = trajx.Trajectory5(0.00, 3,0,0,0,0,0,100);
    //vector<double> y_traj_coefficients = trajy.Trajectory5( -2,1,0,0,0,0,0,100);
   // vector<double> z_traj_coefficients = trajz.Trajectory5(2,3,0,0,0,0,0,100);
   // trajx.init( &x_traj_coefficients[0],6);// position
   // trajy.init( &y_traj_coefficients[0],6);
   // trajz.init( &z_traj_coefficients[0],6);
    
     
   // x velocities and accelerations

   // vector<double> x_dot_traj_coefficients = trajx.polynomial_derivative(x_traj_coefficients);
   // vector<double> x_d_dot_traj_coefficients = trajx.polynomial_derivative(x_dot_traj_coefficients);

     // y velocities and accelerations

   // vector<double> y_dot_traj_coefficients = trajy.polynomial_derivative(y_traj_coefficients);
   // vector<double> y_d_dot_traj_coefficients = trajy.polynomial_derivative(y_dot_traj_coefficients);

	// z velocities and accelerations

    //vector<double> z_dot_traj_coefficients = trajz.polynomial_derivative(z_traj_coefficients);
   // vector<double> z_d_dot_traj_coefficients = trajz.polynomial_derivative(z_dot_traj_coefficients);
    double dt = 0.01;
    ros::Rate loop(1/dt);  
   
    int wait = 0;
    //Parameters for the scurve see: http://www.irccyn.ec-nantes.fr/~briot/PubliConf/DETC2013-13037.pdf;
   double t0 = 0.00000;
   double t1 = 21.3089;
   double t4 = 100.0;
   double t2 = t4/2.0000;
   double t3 = t4 - t1;   
   double ab =  0.0010;
   double vb = 0.0300;
   double hh = 1.1498;
   double hb = 1.5;
   double ZA = 1.5;
   double WA = 2.0;
   double XA = -1.0;


    vector<double> time = trajx.linspace(t0,t4,1000);

for(int i = 0; i< time.size(); i++)
{cout << time[i] << endl;}
position_z.clear();
position_x.clear();
while(iter  < time.size())
{
 cout << "tHE iter " << iter << endl;
  
//Here we are using s-curves :http://www.irccyn.ec-nantes.fr/~briot/PubliConf/D*ETC2013-13037.pdf

if((time[iter]>= t0) && (time[iter] < t1) )
	{    
	  vector<double> z_traj_coefficients = trajz.Trajectory5(0.0,1.0,0.0,vb/hh,0.0,ab/hh,t0,t1);
	double  position = trajz.getposition5(z_traj_coefficients, time[iter]);
         position = (hh*position) + ZA;
	double  velocity = trajz.getvelocity5(z_traj_coefficients, time[iter]);
	 velocity = (hh*velocity);
	double  acceleration = trajz.getacceleration5(z_traj_coefficients, time[iter]);
	 acceleration = (hh*acceleration);
  position_z.push_back(position);
  velocity_z.push_back(velocity); 
  acceleration_z.push_back(acceleration);
   error_sp.positions[2] = position;
   error_sp.velocities[2] = velocity;
   error_sp.accelerations[2] = acceleration;
	}
else if((time[iter] >= t1) && (time[iter] < t2 ))
	{    
	  vector<double> z_traj_coefficients = trajz.Trajectory5(0.0,1.0,vb/(hb-hh),0.0,ab/(hb-hh),0.0,t1,t2);
	 double position = trajz.getposition5(z_traj_coefficients, time[iter]);
	  position = ((hb-hh)*position) + ZA + hh;
	double  velocity = trajz.getvelocity5(z_traj_coefficients, time[iter]);
	  velocity = ((hb- hh)*velocity);
	 double  acceleration = trajz.getacceleration5(z_traj_coefficients, time[iter]);
         acceleration = ((hb- hh)*acceleration);
  position_z.push_back(position);
  velocity_z.push_back(velocity); 
  acceleration_z.push_back(acceleration);
   error_sp.positions[2] = position;
   error_sp.velocities[2] = velocity;
   error_sp.accelerations[2] = acceleration;
	}
else if((time[iter] >= t2) && (time[iter] < t3))
{    
 vector<double> z_traj_coefficients = trajz.Trajectory5(0.0,1.0,0.0,vb/(hb-hh),0.0,-ab/(hb-hh),t2,t3);
  double position = trajz.getposition5(z_traj_coefficients, time[iter]);
 position = position*-(hb-hh) + hb +ZA;
  double velocity = trajz.getvelocity5(z_traj_coefficients, time[iter]);
 velocity = ((-(hb - hh))*velocity);
  double acceleration = trajz.getacceleration5(z_traj_coefficients, time[iter]);
  acceleration = ((-(hb- hh))*acceleration);
  position_z.push_back(position);
  velocity_z.push_back(velocity); 
  acceleration_z.push_back(acceleration);
   error_sp.positions[2] = position;
   error_sp.velocities[2] = velocity;
   error_sp.accelerations[2] = acceleration;
}
else if((time[iter] >= t3) && (time[iter] <= t4) )
{    
vector<double> z_traj_coefficients = trajz.Trajectory5(0.0,1.0,vb/hh,0.0,-ab/hh,0.0,t3,t4);
 double  position = trajz.getposition5(z_traj_coefficients, time[iter]);
 position = ((-hh)*position) + ZA + hh;
  double velocity = trajz.getvelocity5(z_traj_coefficients, time[iter]);
  velocity = ((-hh)*velocity);
 double  acceleration = trajz.getacceleration5(z_traj_coefficients, time[iter]);
 acceleration = ((-hh)*acceleration);
  position_z.push_back(position);
  velocity_z.push_back(velocity); 
  acceleration_z.push_back(acceleration);
   error_sp.positions[2] = position;
   error_sp.velocities[2] = velocity;
   error_sp.accelerations[2] = acceleration;
}


if((time[iter] >= t0) && (time[iter] < t1) )
	{    
	 double  position = (WA/2.0)+XA;
	  double velocity = 0;
	  double acceleration = 0;
  position_x.push_back(position);
  velocity_x.push_back(velocity); 
  acceleration_x.push_back(acceleration);
   error_sp.positions[0] = position;
   error_sp.velocities[0] = velocity;
   error_sp.accelerations[0] = acceleration;
	}
else if((time[iter] >= t1) && (time[iter] < t3) )
	{    
	  vector<double> x_traj_coefficients = trajx.Trajectory5(0.0,1.0,0.0,0.0,0.0,0.0,t1,t3);
	  double position = trajx.getposition5(x_traj_coefficients, time[iter]);
          position = (-WA)*position + (WA/2.0)+XA;
   	 double  velocity = trajx.getvelocity5(x_traj_coefficients, time[iter]);
	  velocity = ((-WA)*velocity);
	  double acceleration = trajx.getacceleration5(x_traj_coefficients, time[iter]);
	 acceleration = ((-WA)*acceleration);
  position_x.push_back(position);
  velocity_x.push_back(velocity); 
  acceleration_x.push_back(acceleration);
   error_sp.positions[0] = position;
   error_sp.velocities[0] = velocity;
   error_sp.accelerations[0] = acceleration;
	}
else if((time[iter] >= t3) && (time[iter] <= t4 ))
{
         double position = (-WA/2.0)+XA;
	  double velocity = 0;
	double   acceleration = 0;
  position_x.push_back(position);
  velocity_x.push_back(velocity); 
  acceleration_x.push_back(acceleration);
   error_sp.positions[0] = position;
   error_sp.velocities[0] = velocity;
   error_sp.accelerations[0] = acceleration;
}
   //cout << "Final trajectory point has been published" << endl;
   
/*
   // x trajectory
  cout << time[iter] << endl;
  position = trajx.getposition5(x_traj_coefficients, time[iter]);
  position_x.push_back(position);
  velocity = trajx.getvelocity5(x_traj_coefficients, time[iter]);
  velocity_x.push_back(velocity);
  acceleration = trajx.getacceleration5(x_traj_coefficients, time[iter]);
  acceleration_x.push_back(acceleration);
   error_sp.positions[0] = position;
   error_sp.velocities[0] = velocity;
   error_sp.accelerations[0] = acceleration;

  // y trajectory
   position = trajy.getposition5(y_traj_coefficients, time[iter]);
   position_y.push_back(position);
   velocity = trajy.getvelocity5(y_traj_coefficients, time[iter]);
   velocity_y.push_back(velocity);
   acceleration = trajy.getacceleration5(y_traj_coefficients, time[iter]);
   cout << "we are here 13  "  <<  acceleration << endl;
   acceleration_y.push_back(acceleration);
   error_sp.positions[1] = position;
   error_sp.velocities[1] = velocity;
   error_sp.accelerations[1] = acceleration;

    // z trajectory
   position = trajz.getposition5(z_traj_coefficients, time[iter]);
   position_z.push_back(position);
   velocity = trajz.getvelocity5(z_traj_coefficients, time[iter]);
   velocity_z.push_back(velocity);
   acceleration = trajz.getacceleration5(z_traj_coefficients, time[iter]);
   acceleration_z.push_back(acceleration);
   error_sp.positions[2] = position;
   error_sp.velocities[2] = velocity;
   error_sp.accelerations[2] = acceleration;
  */

  error_pub.publish(error_sp);
  
//cout << "The desired positions are  . . . xd, yd , zd" << "{  "<< error_sp.positions[0] << "  "<< error_sp.positions[1] << "  "<<error_sp.positions[2] << " }"<<endl;



iter++;
}

cout << "tHE NO OF TIME STEPS IS " << time.size() << endl;
cout << " The total position length " << position_x.size() << endl;

vpPlot plotpx(1, 700, 700, 100, 200, "x-position-Curves...");
vpPlot plotpz(1, 700, 700, 100, 200, "z-position-Curves...");

vpPlot plotvx(1, 700, 700, 100, 200, "x-velocity-Curves...");
vpPlot plotvz(1, 700, 700, 100, 200, "z-velocity-Curves...");
vpPlot plotax(1, 700, 700, 100, 200, "x-acceleration-Curves...");
vpPlot plotaz(1, 700, 700, 100, 200, "z-acceleration-Curves...");
vpPlot plotz(1, 700, 700, 100, 200, "z- Curves...");
vpPlot plotA(1, 700, 700, 100, 200, "S- Curve...");

   // plotx.initGraph(0,0);
   // plotx.initGraph(0,1);
    plotpx.initGraph(0,3);
    plotpz.initGraph(0,3);	
   // ploty.initGraph(0,0);
  // ploty.initGraph(0,1);
    plotvx.initGraph(0,3);
    plotvz.initGraph(0,3);
    plotax.initGraph(0,3);
    plotaz.initGraph(0,3);
   //plotz.initGraph(0,0);
    //plotz.initGraph(0,1);acceleration
    plotz.initGraph(0,3);
//plotz.initGraph(0,1);acceleration
    plotA.initGraph(0,3);
plotpx.initRange (0,0,100,-2,2);
plotA.initRange(0,-2,2,0,4);
plotpz.initRange (0,0,100,0,4);
plotvx.initRange (0,0,100,-0.1,0.1);
plotvz.initRange (0,0,100,-0.1,0.1);
plotax.initRange (0,0,100,-0.01,0.01);
plotaz.initRange (0,0,100,-0.01,0.01);


 
    plotpx.setColor(0,0,vpColor::green);
    plotpx.setColor(0,1,vpColor::red);
    plotpx.setColor(0,2,vpColor::blue);

   plotpz.setColor(0,0,vpColor::green);
   plotpz.setColor(0,1,vpColor::red);
   plotpz.setColor(0,2,vpColor::blue);


    plotvx.setColor(0,0,vpColor::green);
    plotvx.setColor(0,1,vpColor::red);
    plotvx.setColor(0,2,vpColor::blue);

   plotvz.setColor(0,0,vpColor::green);
   plotvz.setColor(0,1,vpColor::red);
   plotvz.setColor(0,2,vpColor::blue);


    plotax.setColor(0,0,vpColor::green);
    plotax.setColor(0,1,vpColor::red);
    plotax.setColor(0,2,vpColor::blue);

   plotaz.setColor(0,0,vpColor::green);
   plotaz.setColor(0,1,vpColor::red);
   plotaz.setColor(0,2,vpColor::blue);

    plotz.setColor(0,0,vpColor::green);
   plotz.setColor(0,1,vpColor::red);
    plotz.setColor(0,2,vpColor::blue);
   
    plotA.setColor(0,0,vpColor::green);
   plotA.setColor(0,1,vpColor::red);
    plotA.setColor(0,2,vpColor::blue);

/*char legend[40];
    strncpy( legend, "position_x", 40 );
    plotpx.setLegend(0,0,legend);
   
    strncpy( legend, "position_z", 40 );
    plotpz.setLegend(0,1,legend);

    char unit[40];
    strncpy( unit, "time(s)", 40 );
    plotpx.setUnitX(1,unit);
    strncpy( unit, "positions (m)", 40 );
    plotpx.setUnitY(1,unit);

    strncpy( unit, "time(s)", 40 );
    plotpz.setUnitX(1,unit);
    strncpy( unit, "positions (m)", 40 );
    plotpz.setUnitY(1,unit);

    strncpy( legend, "velocity_x", 40 );
    plotvx.setLegend(0,0,legend);
    
    strncpy( legend, "velocity_z", 40 );
    plotvz.setLegend(0,1,legend);
    strncpy( unit, "time(s)", 40 );

    plotvx.setUnitX(1,unit);
    strncpy( unit, "velocities (m/s)", 40 );
    plotvx.setUnitY(1,unit);

    plotvz.setUnitX(1,unit);
    strncpy( unit, "velocities (m/s)", 40 );
    plotvz.setUnitY(1,unit);

    strncpy( legend, "acceleration_x", 40 );
    plotax.setLegend(0,0,legend);  
    strncpy( legend, "acceleration_z", 40 );
    plotaz.setLegend(0,1,legend);

    strncpy( unit, "time(s)", 40 );
    plotax.setUnitX(1,unit);
    strncpy( unit, "accelerations (m/s^2)", 40 );
    plotax.setUnitY(1,unit);

    strncpy( unit, "time(s)", 40 );
    plotaz.setUnitX(1,unit);
    strncpy( unit, "accelerations (m/s^2)", 40 );
    plotaz.setUnitY(1,unit);



    
   */
char legend[40];
    strncpy( legend, "S_curve", 40 );
    plotA.setLegend(0,2,legend);
 char unit[40];
    strncpy( unit, "positionsX (m)", 40 );
    plotA.setUnitX(1,unit);
    strncpy( unit, "positionsZ (m)", 40 );
    plotA.setUnitY(1,unit);

  strncpy( legend, "position_x", 40 );
    plotpx.setLegend(0,0,legend);
   
    strncpy( legend, "position_z", 40 );
    plotpx.setLegend(0,1,legend);
    
 plotpx.setUnitX(1,unit);
    strncpy( unit, "positions (m)", 40 );
    plotpx.setUnitY(1,unit);

    strncpy( legend, "velocity_x", 40 );
    plotvx.setLegend(0,0,legend);
    
    strncpy( legend, "velocity_z", 40 );
    plotvx.setLegend(0,1,legend);

    //strncpy( unit, "time(s)", 40 );
    //plotvx.setUnitX(1,unit);
   // strncpy( unit, "velocities (m/s)", 40 );
   // plotvx.setUnitY(1,unit);

    strncpy( legend, "acceleration_x", 40 );
    plotax.setLegend(0,0,legend);  
    strncpy( legend, "acceleration_z", 40 );
    plotax.setLegend(0,1,legend);
    
   // strncpy( unit, "time(s)", 40 );
   // plotax.setUnitX(1,unit);
    //strncpy( unit, "accelerations (m/s^2)", 40 );
   // plotax.setUnitY(1,unit);



 



    
   // vector<string> platform;
   // platform.push_back("cube::platform");
for(int i = 0; i< position_x.size() ; ++i)
{

 //plot.plot(0,0,time[i], positionx[i]);
 plotpx.plot(0,0,time[i], position_x[i]);
 plotpx.plot(0,1,time[i], position_z[i]);

}
 //plotx.plot(0,1,time[i], position_y[i]);
for(int i = 0; i< position_x.size() ; ++i)
{
 plotpz.plot(0,1,time[i], position_z[i]);
 plotA.plot(0,2,position_x[i], position_z[i]);
 //plot.plot(2,0,time[i], forcex[i]);
 //plot.plot(3,0,time[i],std::abs(positionx[i] -  0));

 }
for(int i = 0; i<  velocity_x.size() ; ++i){
 plotvx.plot(0,0,time[i], velocity_x[i]);
 plotvx.plot(0,1,time[i], velocity_z[i]);
 //ploty.plot(0,1,time[i], velocity_y[i]);
}
for(int i = 0; i<  velocity_z.size() ; ++i)
{
 plotvz.plot(0,1,time[i], velocity_z[i]);
}

for(int i = 0; i<  acceleration_x.size() ; ++i)
{
 plotax.plot(0,0,time[i], acceleration_x[i]);
 plotax.plot(0,1,time[i], acceleration_z[i]);
// plotz.plot(0,1,time[i], acceleration_y[i]);
}
for(int i = 0; i<  acceleration_z.size(); ++i)
{
 plotaz.plot(0,1,time[i], acceleration_z[i]);
//
}


// plot.plot(0,1,time[i],positiony[i]);
// plot.plot(1,1,time[i], position_y[i]);
 //plot.plot(2,1,time[i],forcey[i]);
 //plot.plot(3,1,time[i],std::abs(positiony[i] -0));

 //plot.plot(0,2,time[i],positionz[i]);
 //plot.plot(1,2,time[i],position_z[i]);
// plot.plot(2,2,time[i],forcez[i]);
 //plot.plot(2,3,time[i],forcew[i]);
 //plot.plot(3,2,time[i],std::abs(positionz[i] - 3));




std::getc(stdin);





 cout << "CDPR control ready" << fixed << endl;
}

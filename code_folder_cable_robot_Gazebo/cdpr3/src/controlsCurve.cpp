
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
double position, velocity, acceleration;

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
   force_sub2 = nh.subscribe("cdpr_state", 10000, &Platformcb);
   

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
    ros::Publisher error_pub = nh.advertise< trajectory_msgs::JointTrajectoryPoint >("trajectory_error", 1000);
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
    cout << "t[0] is " << t_[0] << endl;
    cout << "t[1] is " << t_[1] << endl;
    cout << "t[2] is " << t_[2] << endl;
   // vector<double> x_traj_coefficients = trajx.Trajectory5(0.00, 3,0,0,0,0,0,100);
    //vector<double> y_traj_coefficients = trajy.Trajectory5( -2,1,0,0,0,0,0,100);
   // vector<double> z_traj_coefficients = trajz.Trajectory5(2,3,0,0,0,0,0,100);
   // trajx.init( &x_traj_coefficients[0],6);// position
   // trajy.init( &y_traj_coefficients[0],6);
   // trajz.init( &z_traj_coefficients[0],6);
    
     
     double dt = 0.01;
    ros::Rate loop(1/dt);  
    int wait = 0;
    //vector<double> position_vector, velocity_vector, acceleration_vector;
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
   
    
    //vector<double> position_vector, velocity_vector, acceleration_vector;

while(ros::ok()){
 
  
  if(message_recieved && iter != time.size())
{
   
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

  error_sp.positions[1] = 0;
   error_sp.velocities[1] = 0;
   error_sp.accelerations[1] = 0;
error_pub.publish(error_sp);

cout << "The desired positions are  . . . xd, yd , zd" << "{  "<< error_sp.positions[0] << "  "<< error_sp.positions[1] << "  "<<error_sp.positions[2] << " }"<<endl;

  
iter++;

}else
{

cout << "Published the last trajectory" << endl;
cout << "tHE NO OF TIME STEPS IS " << time.size() << endl;
cout << " The total position length " << positionx.size() << endl;


vpPlot plotx(1, 700, 700, 100, 200, "x-Curves... ");
vpPlot ploty(1, 700, 700, 100, 200, "y-Curves...");
vpPlot plotz(1, 700, 700, 100, 200, "z- Curves...");
vpPlot plot(1, 700, 700, 100, 200, "Curves...");
vpPlot plotA(1, 700, 700, 100, 200, "Curves...");
vpPlot plote(1, 700, 700, 100, 200, "Curves...");

	plotx.initGraph(0,3);
	ploty.initGraph(0,3);
	plotz.initGraph(0,3);
	plot.initGraph(0,n);
        plotA.initGraph(0,1);
        plote.initGraph(0,1);

 
    plotx.setColor(0,0,vpColor::green);
    plotx.setColor(0,1,vpColor::red);
    plotx.setColor(0,2,vpColor::blue);

   ploty.setColor(0,0,vpColor::green);
   ploty.setColor(0,1,vpColor::red);
   ploty.setColor(0,2,vpColor::blue);

    plotz.setColor(0,0,vpColor::green);
    plotz.setColor(0,1,vpColor::red);
    plotz.setColor(0,2,vpColor::blue);

    plotA.setColor(0,0,vpColor::red);
    plote.setColor(0,0,vpColor::blue);


    char legend[40];
    strncpy( legend, "position_x", 40 );
    plotx.setLegend(0,0,legend);

    strncpy( legend, "position_z", 40 );
    plotx.setLegend(0,1,legend);
    char unit[40];
    strncpy( unit, "time(s)", 40 );
    plotx.setUnitX(1,unit);
    strncpy( unit, "positions (m)", 40 );
    plotx.setUnitY(1,unit);


    strncpy( legend, "x position error", 40 );
    ploty.setLegend(0,0,legend);
   
    strncpy( legend, "z position error", 40 );
    ploty.setLegend(0,1,legend);

    strncpy( unit, "time(s)", 40 );
    ploty.setUnitX(1,unit);
    strncpy( unit, "position (m)", 40 );
    ploty.setUnitY(1,unit);

    strncpy( legend, "acceleration_x", 40 );
    plotz.setLegend(0,0,legend);
 
    strncpy( legend, "acceleration_z", 40 );
    plotz.setLegend(0,1,legend);

    strncpy( unit, "time(s)", 40 );
    plotz.setUnitX(1,unit);
    strncpy( unit, "accelerations (m/s^2)", 40 );
    plotz.setUnitY(1,unit);


    strncpy( unit, "time(s)", 40 );
    plot.setUnitX(1,unit);
    strncpy( unit, "Tensions (N)", 40 );
    plot.setUnitY(1,unit);

    strncpy( legend, "error in scurve", 40 );
    plote.setLegend(0,0,legend);

    strncpy( legend, "Actual S-curve", 40 );
    plotA.setLegend(0,0,legend);



	for(int i =0 ; i< n; ++i)
	{

	plot.setColor(0,i,vpColor::getColor(i+3));

	}
    char title[40];
    strncpy( title, " Positions error ", 40 );
    plotx.setTitle(0,title);   
    strncpy( title, " Actual Velocities  ", 40 );
    ploty.setTitle(0,title); 
   strncpy( title, " Desired Position ", 40 );
    plotz.setTitle(0,title);
    strncpy( title, " Applied Forces ", 40 );
    plot.setTitle(0,title);

 
   int offset = time.size() -positionx.size(); 

for(int i =  0; i< positionx.size(); ++i)
{

 plotx.plot(0,0,time[i+offset],     positionx[i] ); 
 plotA.plot(0,0, positionx[i],   positionz[i] );
plote.plot(0,0, position_x[i+offset] - positionx[i], position_z[i+offset] - positionz[i]    ); 
 plotx.plot(0,1,time[i+offset],    positionz[i] );
 
 ploty.plot(0,0, time[i+offset],position_x[i+offset] - positionx[i]);
 ploty.plot(0,1, time[i+offset], position_z[i+offset] - positionz[i]);
 //ploty.plot(0,1,time[i+offset],  velocity_x[i]);
 //ploty.plot(0,2,time[i+offset],  velocity_z[i]);


 //plotz.plot(0,0,time[i+offset],  positionz[i]);
 //plotz.plot(0,0,time[i], accelerationx[i]);
 //plotz.plot(0,1,time[i], accelerationz[i] );
 //plotz.plot(0,1, positionx[i],   positionz[i] );

}
offset = time.size() - forces.size(); 
cout << " This is forces.size()" << forces.size() << endl;
cout << " This is offset" << offset << endl;
for(int i= 0; i < forces.size();i++)
	for(int j= 0; j < n;j++)
	{
		{ 
		 plot.plot(0,j,time[i+offset], forces[i][j]);
 biggest.push_back(*max_element(forces[i].begin(), forces[i].end()));
		 
		}
	}
cout << "The max force is " <<   *max_element(biggest.begin(), biggest.end()) << endl;

std::getc(stdin);

}

     ros::spinOnce();
     loop.sleep();



 cout << "CDPR control ready" << fixed << endl;

}
}

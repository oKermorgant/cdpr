
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
    vector<double> x_traj_coefficients = trajx.Trajectory5(0.00, 2,0,0,0,0,0,100);
    vector<double> y_traj_coefficients = trajy.Trajectory5( 0.00,2,0,0,0,0,0,100);
    vector<double> z_traj_coefficients = trajz.Trajectory5(1.5,3,0,0,0,0,0,100);
    trajx.init( &x_traj_coefficients[0],6);// position
    trajy.init( &y_traj_coefficients[0],6);
    trajz.init( &z_traj_coefficients[0],6);
    
     
   // x velocities and accelerations
    vector<double> time = trajx.linspace(0,100,1000);
    vector<double> x_dot_traj_coefficients = trajx.polynomial_derivative(x_traj_coefficients);
    vector<double> x_d_dot_traj_coefficients = trajx.polynomial_derivative(x_dot_traj_coefficients);

     // y velocities and accelerations

    vector<double> y_dot_traj_coefficients = trajy.polynomial_derivative(y_traj_coefficients);
    vector<double> y_d_dot_traj_coefficients = trajy.polynomial_derivative(y_dot_traj_coefficients);

	// z velocities and accelerations

    vector<double> z_dot_traj_coefficients = trajz.polynomial_derivative(z_traj_coefficients);
    vector<double> z_d_dot_traj_coefficients = trajz.polynomial_derivative(z_dot_traj_coefficients);
    double dt = 0.01;
    ros::Rate loop(1/dt);  
   
    
    //vector<double> position_vector, velocity_vector, acceleration_vector;

while(ros::ok()){
 
  
  if(message_recieved && iter != time.size())
{
   
   //cout << "Final trajectory point has been published" << endl;
   
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

	plotx.initGraph(0,3);
	ploty.initGraph(0,3);
	plotz.initGraph(0,3);
	plot.initGraph(0,n);
 
    plotx.setColor(0,0,vpColor::green);
    plotx.setColor(0,1,vpColor::red);
    plotx.setColor(0,2,vpColor::blue);

   ploty.setColor(0,0,vpColor::green);
   ploty.setColor(0,1,vpColor::red);
   ploty.setColor(0,2,vpColor::blue);

    plotz.setColor(0,0,vpColor::green);
    plotz.setColor(0,1,vpColor::red);
    plotz.setColor(0,2,vpColor::blue);

    char legend[40];
    strncpy( legend, "position_x", 40 );
    plotx.setLegend(0,0,legend);
    strncpy( legend, "position_y", 40 );
    plotx.setLegend(0,1,legend);
    strncpy( legend, "position_z", 40 );
    plotx.setLegend(0,2,legend);
    char unit[40];
    strncpy( unit, "time(s)", 40 );
    plotx.setUnitX(1,unit);
    strncpy( unit, "positions (m)", 40 );
    plotx.setUnitY(1,unit);


    strncpy( legend, "velocity_x", 40 );
    ploty.setLegend(0,0,legend);
    strncpy( legend, "velocity_y", 40 );
    ploty.setLegend(0,1,legend);
    strncpy( legend, "velocity_z", 40 );
    ploty.setLegend(0,2,legend);
    strncpy( unit, "time(s)", 40 );
    ploty.setUnitX(1,unit);
    strncpy( unit, "velocities (m)", 40 );
    ploty.setUnitY(1,unit);

    strncpy( legend, "acceleration_x", 40 );
    plotz.setLegend(0,0,legend);
    strncpy( legend, "acceleration_y", 40 );
    plotz.setLegend(0,1,legend);
    strncpy( legend, "acceleration_z", 40 );
    plotz.setLegend(0,2,legend);
    strncpy( unit, "time(s)", 40 );
    plotz.setUnitX(1,unit);
    strncpy( unit, "accelerations (m)", 40 );
    plotz.setUnitY(1,unit);

    strncpy( unit, "time(s)", 40 );
    plot.setUnitX(1,unit);
    strncpy( unit, "Tensions (N)", 40 );
    plot.setUnitY(1,unit);


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

 plotx.plot(0,0,time[i+offset],    positionx[i] ); 
 plotx.plot(0,1,time[i+offset],    positiony[i] );
 plotx.plot(0,2,time[i+offset],    positionz[i] );
 
 ploty.plot(0,0,time[i+offset],  position_x[i]);
 //ploty.plot(0,1,time[i+offset],  position_y[i]);
 ploty.plot(0,2,time[i+offset],  position_z[i]);
 //ploty.plot(0,2,time[i+offset],  3 - position_z[i]);


 //plotz.plot(0,0,time[i], acceleration_x[i]);
 //plotz.plot(0,1,time[i], acceleration_y[i]);
 //plotz.plot(0,2,time[i],acceleration_z[i] );
 plotz.plot(0,2,time[i+offset],  3 - positionz[i]);

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

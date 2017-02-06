#include <cdpr3/cdpr_plugin.h>

namespace gazebo
{
void CableDrivenParallelRobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get model and name
    model_ = _model;
    robot_namespace_ = model_->GetName();
  cout << "this is the namespace"<< robot_namespace_ << endl;
    // register ROS node & time
    rosnode_ = ros::NodeHandle(robot_namespace_);
    //ros::NodeHandle control_node(rosnode_, "controllers");
    t_prev_= 0;

    // check for body or joint param
    control_body_ = false;
    control_joints_ = false;
 
  
    while(!(control_body_ || control_joints_))
    {
        control_body_ =  rosnode_.hasParam("controllers/config/body");
        control_joints_ =  rosnode_.hasParam("controllers/config/joints");
    }

	  if(control_joints_ && model_->GetJointCount() != 0)
    {
    //cout << "	This sis the sdf name" << _sdf->GetName()<< "DESCRIPTRION --"<< _sdf->GetDescription() <<  endl;
       body_ = model_->GetLink("platform");
       cout << "this is the name of the model"<< body_->GetName() << endl;
   
    std::string body_command_topic, body_state_topic;
      if(_sdf->HasElement("link ")){ cout << "Yes!! 0" << endl;}

        rosnode_.param("controllers/config/body/command", body_command_topic, std::string("body_command"));
        rosnode_.param("controllers/config/body/state", body_state_topic, std::string("body_state"));

        if(_sdf->HasElement("link"))
	  {
            body_ = model_->GetLink(_sdf->Get<std::string>("link"));
            mass = body_->GetInertial()->GetMass();
           cout << "Successfully loaded the platform link object " << body_->GetName() << endl;
          }
        else
           { 
		body_ = model_->GetLink("platform");
 		mass = body_->GetInertial()->GetMass();
		cout << "Tried to get SDF Link. . . . .There could be problems, Using an alternative function GetLinks()"  << "we found this mass "<<mass << endl;        	    }
        
          // Initialize Subscriber to Trajectory error
     rosnode_.param("controllers/config/trajectory/error", trajectory_command_topic, std::string("trajectory_error")); 
  rosnode_.param("controllers/config/trajectory/forcematlab", forceMatlab_command_topic, std::string("forces_matlab")); 
   cout <<" we are here 4 " << endl;
ros::SubscribeOptions ops2 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                    forceMatlab_command_topic, 1,
                    boost::bind(&CableDrivenParallelRobotPlugin::TrajectoryCommandCallBackMA, this,_1),
                    ros::VoidPtr(), &callback_queue_);
cout <<" we are here 4.5 " << endl;
ros::SubscribeOptions ops3 = ros::SubscribeOptions::create<trajectory_msgs::JointTrajectoryPoint>(
                    trajectory_command_topic, 1,
                    boost::bind(&CableDrivenParallelRobotPlugin::TrajectoryCommandCallBackRS, this,_1),
                    ros::VoidPtr(), &callback_queue_);

cout <<" we are here 4.6 " << endl;

    /*control_error_subscriber_ = rosnode_.subscribe<std_msgs::Float32MultiArray>( trajectory_command_topic, 1,boost::bind(&CableDrivenParallelRobotPlugin::TrajectoryCommandCallBack, _1,this));*/


  control_MATLAB_subscriber_ = rosnode_.subscribe(ops2);
cout <<" we are here 4.7 " << endl;
 control_ROS_subscriber_ = rosnode_.subscribe(ops3);

//control_error_subscriber_ = rosnode_.subscribe( trajectory_command_topic, 1,TrajectoryCommandCallBack);
   cout <<" we are here 5 " << endl;
 	// initialize the paramters
        //gravity
	g.resize(3,true);
        // Tension to be applies             
        u.resize(8);
      	g[2] = 9.8;
        //Min and Max tENSION limits
	fmax.resize(8,true);
	fmin.resize(8,true);
        f_v3.resize(8,true);
        // Structure Matrix
	T.resize(3,8);
 	//Gain parameters
	 Kp.resize(3,3);
	 Kd.resize(3,3);
   	feasible_Index_set.resize(4);
	//feasible_Index_set.resize(3);

//Minimun and Maximum Forces
for(unsigned int i = 0; i < fmax.size(); i++)
{
    fmax[i] = 6500;
    fmin[i] = 1;
}
	
//Matrixes for performing vector orthogonal transform [a b]=> [-b a]
 pros = vpMatrix(2,2,0);
pros[0][1] = -1;
pros[1][0] = 1;

 pross = vpMatrix(2,2,0);
pross[0][1] = 1;
pross[1][0] = -1;

countCables = 0;

//Matrix for indices
 index_matrix = vpMatrix(4,4,0);
	
        // cable attach points
    XmlRpc::XmlRpcValue points, point_s;
   cout <<" we are here  6.5 " << endl;
    rosnode_.getParam("model/points", points);
   cout <<" we are here 7 " << endl;
     cout << "The number of points is" << points.size() << endl;
     cout <<" we are here 3 " << endl;
       for(unsigned int i=0;i<points.size();++i)
    {

    
         x = points[i]["frame"][0];
         y= points[i]["frame"][1];
         z = points[i]["frame"][2];
         frame_point.push_back(vpTranslationVector(x, y, z));
         x = points[i]["platform"][0];
         y= points[i]["platform"][1];
         z = points[i]["platform"][2];
         anchor_point.push_back(vpTranslationVector(x, y, z));
cout << "we are here 1" << endl;
    }
cout << "The size of frame p after is " << frame_point.size() << endl;
    rosnode_.getParam("model/global_frame", point_s);
  rosnode_.setParam("controllers/config/body/link", body_->GetName());

    
    // *** JOINT CONTROL
    joints_.clear();

  

        std::string joint_command_topic, joint_state_topic;
        rosnode_.param("controllers/config/joints/command", joint_command_topic, std::string("cable_command"));
        rosnode_.param("controllers/config/joints/state", joint_state_topic, std::string("cable_states"));

        // initialize subscriber to joint commands
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    joint_command_topic, 1,
                    boost::bind(&CableDrivenParallelRobotPlugin::JointCommandCallBack, this, _1),
                    ros::VoidPtr(), &callback_queue_);
        joint_command_subscriber_ = rosnode_.subscribe(ops);
        joint_command_received_ = false;

        // push joint limits and setup joint states
        std::vector<std::string> joint_names;
        std::vector<double> joint_min, joint_max;
        std::string name, namex;
        physics::JointPtr joint;
        joints_.clear();
        bool cascaded_position = true;
        if(rosnode_.hasParam("controllers/config/joints/cascaded_position"))
            rosnode_.getParam("controllers/config/joints/cascaded_position", cascaded_position);


        for(int i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();
            namex = joint->GetName(); 
            namex.insert(0,"controllers/");
            cout << "This is name " << name << endl;
            cout << "This is namex " << namex << endl;
            if(rosnode_.hasParam(namex))
            {
		cout << "we have this joint " << name <<  endl; 
                joints_.push_back(joint);
                // set max velocity or max effort for the position PID
                sprintf(param, "%s/position/i_clamp", name.c_str());
                if(cascaded_position)
                    rosnode_.setParam(param, joint->GetVelocityLimit(0));
                else
                   rosnode_.setParam(param, joint->GetEffortLimit(0));

                // set max effort for the velocity PID
                sprintf(param, "%s/velocity/i_clamp", name.c_str());
                rosnode_.setParam(param, joint->GetEffortLimit(0));

                // save name and joint limits
                joint_names.push_back(name);
                joint_min.push_back(joint->GetLowerLimit(0).Radian());
                joint_max.push_back(joint->GetUpperLimit(0).Radian());
            }
        }

        // push setpoint topic, name, lower and bound
        rosnode_.setParam("controllers/config/joints/name", joint_names);
        rosnode_.setParam("controllers/config/joints/lower", joint_min);
        rosnode_.setParam("controllers/config/joints/upper", joint_max);

        // setup joint_states publisher
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
        
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
         
        
        platform_state_publisher_ = rosnode_.advertise<nav_msgs::Odometry>("cdpr_state", 1000);
       platform_force_publisher_ = rosnode_.advertise< std_msgs::Float64MultiArray>("applied_force_record", 1000);
    }
    // *** END JOINT CONTROL
//SERVICE CLIENT
       solverClient = rosnode_.serviceClient<cdpr3::SOLVEQP>("SOLVETheQP");
  

     // store update rate
    if(_sdf->HasElement("updateRate"))

        update_T_ = 1./_sdf->Get<double>("updateRate");
        
    else
        update_T_ = 0;

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CableDrivenParallelRobotPlugin::Update, this));
    
    controller_is_running_ = true;
static_control = true;
     u_f = staticEquilibrum();
   // Compute The initial polygon interserection based on the initial pose
      ros::spinOnce();
    ROS_INFO("Started FreeFloating Control Plugin for %s.", _model->GetName().c_str());
}

void CableDrivenParallelRobotPlugin::Update()
{
    // activate callbacks
    callback_queue_.callAvailable();
    countCables++;

    if(controller_is_running_)
    {
        // deal with joint control
        if(control_joints_ && joint_command_received_)
        {
            physics::JointPtr joint;
            for(unsigned int i=0;i<joints_.size();++i)
            {
              cout << "the size of the joints is"  << joints_.size()  << endl;
             cout << "the name of the joint is"  << joints_[i]->GetName() << endl;
                joint = joints_[i];
                //joint->SetForce(0,joint_command_.effort[i] - joint->GetDamping(0) * joint->GetVelocity(0));
		//joints_[i]->SetForce(0,0);
                joints_[i]->SetForce(0,u[i]);
            cout << "This is the value of the force as given by the joint"  << joints_[i]->GetForce(0)<< endl;
            cout << "This is the value of the Damping as given by the joint"  << joints_[i]->GetDamping(0)<< endl;

              }
  joint_command_received_ = false;
        }


if(static_control)
	{   
cout << " the update is being done here " << u_f.t() << endl;
		    physics::JointPtr joint;
		    for(unsigned int i=0;i<joints_.size();++i)
		    {
		      cout << "the size of the joints is"  << joints_.size()  << endl;
		     cout << "the name of the joint is"  << joints_[i]->GetName() << endl;
		        joint = joints_[i];
		        //joint->SetForce(0,joint_command_.effort[i] - joint->GetDamping(0) * joint->GetVelocity(0));
			//joints_[i]->SetForce(0,0);
		        joints_[i]->SetForce(0,u_f[i]);
		    cout << "This is the value of the force as given by the joint"  << joints_[i]->GetForce(0)<< endl;
		    cout << "This is the value of the Damping as given by the joint"  << joints_[i]->GetDamping(0)<< endl;
		    
		    }

	} 



       
   }

    // publish joint states anyway
    double t = ros::Time::now().toSec();
    if((t-t_prev_) > update_T_ && joints_.size() != 0)
    {
        t_prev_ = t;
        joint_states_.header.stamp = ros::Time::now();

        for(unsigned int i=0;i<joints_.size();++i)
        {
           
            

	    joint_states_.position[i] = joints_[i]->GetAngle(0).Radian();
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
        }
        joint_state_publisher_.publish(joint_states_);
         
    }



}


void CableDrivenParallelRobotPlugin::TrajectoryCommandCallBackMA(const std_msgs::Float32MultiArrayConstPtr _msg)
{
cout << "we have recieved a message" <<endl;
int cableno = _msg->layout.data_offset; 
cout << "This is the data_offset" << cableno <<endl;
 u.resize(cableno, true);


for(unsigned int i=0;i<cableno;++i)
	{
u[i] = _msg->data[i];
cout << "we are here " << i << endl; 
	}

cout << "These are the forces we sent " << u.t() << endl;
joint_command_received_ = true;
}









void CableDrivenParallelRobotPlugin::TrajectoryCommandCallBackRS(const trajectory_msgs::JointTrajectoryPointConstPtr  &_msg)
{
  
    desired_position = vpTranslationVector(_msg->positions[0],_msg->positions[1], _msg->positions[2]);
    desired_rotation = vpRotationMatrix(vpThetaUVector(0,0,0));
         
    desired_velocity = vpTranslationVector(_msg->velocities[0],_msg->velocities[1], _msg->velocities[2]);
    desired_rotation_d = vpRotationMatrix(vpThetaUVector(0,0,0));
       
    desired_acceleration = vpTranslationVector(_msg->accelerations[0],_msg->accelerations[1], _msg->accelerations[2]);
    desired_rotation_dd = vpRotationMatrix(vpThetaUVector(0,0,0));

    platform_pose = body_->GetWorldPose();
    platform_vel = body_->GetWorldLinearVel();
    platform_ang = body_->GetWorldAngularVel();


  current_position = vpTranslationVector(platform_pose.pos.x,platform_pose.pos.y, platform_pose.pos.z);
  current_rotation = vpRotationMatrix(vpQuaternionVector(platform_pose.rot.x,platform_pose.rot.y,platform_pose.rot.z,platform_pose.rot.w));
  
    current_velocity = vpTranslationVector(platform_vel.x,platform_vel.y, platform_vel.z);
    current_velocity_d = vpThetaUVector(platform_ang.x,platform_ang.y, platform_ang.z);
cout << "This is the current_position " << current_position.t() << endl;
cout << "This is the current_position_d " << current_rotation.t() << endl;

cout << "This is the current_velocity " << current_velocity.t() << endl;
cout << "This is the current_velocity_d " << current_velocity_d.t() << endl;
cout << "This is the desired acceleration " << desired_acceleration.t() << endl;

    //Errors in end effector position, velocity and acceleration
   // x_error =     current_rotation*(current_position - desired_position); 
    //x_dd_error =   current_velocity - desired_velocity;
   vpMatrix fRRp(6,6);
    //xdd =  desired_acceleration;
  	for(unsigned int i=0;i<3;++i)
                 for(unsigned int j=0;j<3;++j)
                     fRRp[i][j] = fRRp[i+3][j+3] =  current_rotation[i][j];
fRRp.matlabPrint(std::cout);

 vpHomogeneousMatrix Mp, Mpd, Vp, Vpd;
 Mp.insert(current_position);
 Mp.insert(current_rotation);
 std::cout << "Mp:\n" << Mp << std::endl;

 Mpd.insert(desired_position);
 Mpd.insert(desired_rotation);
 std::cout << "Mpd:\n" << Mpd << std::endl;

 Vp.insert(current_velocity);
 Vp.insert(current_velocity_d);
 std::cout << "Vp:\n" << Vp << std::endl;

 Vpd.insert(desired_velocity);
 Vpd.insert(desired_rotation_d);
 std::cout << "Vpd:\n" << Vpd << std::endl;

// position error
x_error = desired_position - current_position ;

// velocity error
x_dd_error =  desired_velocity - current_velocity ; 

//desired acceleration
xdd = desired_acceleration;
cout << "This is calculated xdd" << xdd.t() << endl;

cout << "This is calculated xdd" << xdd.t() << endl;

	// Assign the gain for use in this calculation
	Kp.diag(500);
	Kd.diag(500);
	Kd.matlabPrint(std::cout);
	Kp.matlabPrint(std::cout);

      // build T matrix depending on current attach points
          for(unsigned int i=0;i<frame_point.size();++i)
     	{   
       
                // vector between desired platform point and frame point in platform frame
                f =    current_position + (current_rotation*anchor_point[i])- frame_point[i]  ;
                f /= f.euclideanNorm();
       cout << "This is the f" << f.t() << endl;
                // Uncomment w if we control rotations to form a 6 x 8 system
               // w = ((current_rotation*anchor_point[i]).skew() *( current_position  - frame_point[i]))/( current_position + (current_rotation*anchor_point[i])- frame_point[i])..euclideanNorm() ;
            //cout << "This is the w" << w.t() << endl;
		

                for(unsigned int k=0;k<3;++k)
                {
                    T[k][i] = f[k];
                    //T[k+3][i] = w[k];

		}
             
 	 }
        //Output Variables used for this computation on the terminal
	cout << "This is calculated gravity" << g.t() << endl;
	cout << "This is calculated xdd" << xdd.t() << endl;
	 cout << "This is calculated x_error" << x_error.t() << endl;
	  cout << "This is calculated x_dd_error" << x_dd_error.t() << endl;
       
//output wrench matrix to terminal
T.matlabPrint(std::cout);



//Errror as a wrench: see :http://kth.diva-portal.org/smash/get/diva2:864071/FULLTEXT01.pdf 
w_vector = mass*(xdd + Kp*x_error + Kd*x_dd_error + g);


 cout << "This is calculated w_vector" << w_vector.t() << endl;



// Compute Basic Solution with Pseudo inverse, some tensions might be negative
f_v = -T.pseudoInverse()*w_vector;





//Uncomment if we are using QP or LP method. Think to select method automatically from dropdown in GUI	

//w_vector = -w_vector;
//f_v = ActivesetMethod();


//Uncomment if we are using closed form method 
//f_m = (fmax + fmin)/2;
//f_v = f_m  - (T.pseudoInverse()*(w_vector + (T*f_m)));


//Uncomment when we want to use the service call method to call CGAL
	
/*cdpr3::SOLVEQP serv;
serv.request.WrenchMatrix.clear();
serv.request.wVector.clear();
	

for(int i = 0; i<3; i++)
{ serv.request.wVector.push_back(w_vector.data[i]); 
cout << "Pushing back wvector " << i<< endl;
 }

for(int i = 0; i<24; i++)
{ serv.request.WrenchMatrix.push_back(T.data[i]);

cout << "Pushing back WrenchMatrix " << i << endl;}

  if(solverClient.call(serv))
  {
    	ROS_INFO("Suceeded in calling solver");
 	f_v.resize(8,true);
	for(int i = 0; i<8; i++)
		{f_v[i] = serv.response.tensions[i];
cout << "Pushing back f_v tensions " << i << endl;
ROS_INFO("Suceeded in filling tensions solver");}
  }
  else
  {
    ROS_ERROR("Failed to call service Tension distribution");
   f_v = fmin;
  }


//Uncomment when we want to simulate broken cables
/*
if(countCables >60)
{
f_v[1] = 0;
f_v[2] = 0;
f_v[3] = 0;
f_v[4] = 0;
f_v[5] = 0;
}*/

// Assign calculated forces to the tension variable
u = f_v;  



//u = f_v+T_Kernel*v_init;   
		
    // Create a force message to be sent to the plotter
    applied_force.data.clear();
   for (int i = 0; i < u.size(); i++)
		{
	       applied_force.data.push_back(u[i]);
		}

   
    // Get the resulting end effector position to be sent as message
    //peace = body_->GetWorldPose();
    //platform_state.x = peace.pos.x;
    //platform_state.y = peace.pos.y;
    //platform_state.z = peace.pos.z;

    // Publish the applied force and the resulting position
    //platform_pose_publisher_.publish(applied_force);
    //platform_state_publisher_.publish(platform_state);
             
        platform_force_publisher_.publish(applied_force); 
// Get the resulting end effector position to be sent as message
   // platform_pose = body_->GetWorldPose();
    //platform_vel = body_->GetWorldLinearVel();
    //platform_ang = body_->GetWorldAngularVel();

    //platform_state.name[0] = "platform";

    platform_state.pose.pose.position.x = platform_pose.pos.x;
    platform_state.pose.pose.position.y = platform_pose.pos.y;
    platform_state.pose.pose.position.z = platform_pose.pos.z;

    platform_state.pose.pose.orientation.x = platform_pose.rot.x;
    platform_state.pose.pose.orientation.y = platform_pose.rot.y;
    platform_state.pose.pose.orientation.z = platform_pose.rot.z;
    platform_state.pose.pose.orientation.w = platform_pose.rot.w;

//fill the twist message



   platform_state.twist.twist.linear.x = platform_vel.x;
     platform_state.twist.twist.linear.y = platform_vel.y;
    platform_state.twist.twist.linear.z = platform_vel.z;
    platform_state.twist.twist.angular.x = platform_ang.x;
     platform_state.twist.twist.angular.y = platform_ang.y;
    platform_state.twist.twist.angular.z = platform_ang.z;


    // Publish  the resulting pose data
 platform_state_publisher_.publish(platform_state);

       joint_command_received_ = true;

cout << "the applied forces are  " << "[ " << u.t() << " ]" << endl;

}










 vpColVector CableDrivenParallelRobotPlugin::staticEquilibrum()
{

 platform_pose = body_->GetWorldPose();



  current_position = vpTranslationVector(platform_pose.pos.x,platform_pose.pos.y, platform_pose.pos.z);
  current_rotation = vpRotationMatrix(vpQuaternionVector(platform_pose.rot.x,platform_pose.rot.y,platform_pose.rot.z,platform_pose.rot.w));
  
cout << "This is the current_position " << current_position.t() << endl;
cout << "This is the current_position_d " << current_rotation.t() << endl;

   // Assign the gain for use in this calculation
	Kp.diag(50);
	Kd.diag(50);
	Kd.matlabPrint(std::cout);
	Kp.matlabPrint(std::cout);
      // build T matrix depending on current attach points
          for(unsigned int i=0;i<frame_point.size();++i)
     	{   
       
                // vector between desired platform point and frame point in platform frame
                f =    current_position + (current_rotation*anchor_point[i])- frame_point[i]  ;
                f /= f.euclideanNorm();
       cout << "This is the f" << f.t() << endl;
                
                for(unsigned int k=0;k<3;++k)
                {
                    T[k][i] = f[k];
                    //T[k+3][i] = w[k];

		}
             
 	 }
        //Output Variables used for this computation on the terminal
	cout << "This is calculated gravity" << g.t() << endl;


T.matlabPrint(std::cout);

w_vector = mass*(g); 
cout << "This is calculated w_vector" << w_vector.t() << endl;


// Compute Basic Solution with Pseudo inverse, some tensions might be negative
//f_v = -T.pseudoInverse()*w_vector;	

w_vector = -w_vector;
f_v = ActivesetMethod();
return f_v;

}




vpColVector CableDrivenParallelRobotPlugin::ActivesetMethod()
{

// We make use of qpOASES solver 

   
//get number of cables

int no_of_cables = T.getCols();
vpMatrix hessiann(T.getCols(),T.getCols());

//gradient vector and hessian for use in lP
//hessiann.diag(0);
//vpColVector grad(T.getCols(),1.0);


//gradient vector and Hessian for use in QP
hessiann.diag(1);
vpColVector grad(T.getCols(),0.0);

real_t H[T.getCols()*T.getCols()];
real_t A[T.getRows()*T.getCols()];
real_t grad_vec[T.getCols()];
real_t lb[T.getCols()];
real_t ub[T.getCols()];
real_t lbA[T.getRows()];
real_t ubA[T.getRows()];

for (int i = 0; i<(T.getCols()*T.getCols()); i++)
{ 
H[i] = hessiann.data[i];
}

hessiann.matlabPrint(std::cout);
for (int i = 0; i<(T.getRows()*T.getCols()); i++)
{ 
A[i] = T.data[i]; 
}
T.matlabPrint(std::cout);
for (int i = 0; i<(T.getCols()); i++)
{ 
grad_vec[i] = grad.data[i];
lb[i] = fmin.data[i];
ub[i] = fmax.data[i];
}

for (int i = 0; i<(T.getRows()); i++)
{ 
lbA[i] = w_vector.data[i];
ubA[i] = w_vector.data[i];
}

// For use in LP
     //QProblem example( T.getCols(),T.getRows(),HST_ZERO);
// For use in qP
    QProblem example( T.getCols(),T.getRows(),HST_IDENTITY);


	/* Set the number of working set re-calculations*/ 
	int_t nWSR = 1000;
// For use in LP
//returnValue statusCode = example.init( 0,grad_vec,A,lb,ub,lbA,ubA, nWSR,0);
// For use in QP
returnValue statusCode = example.init( H,grad_vec,A,lb,ub,lbA,ubA, nWSR,0);
if(statusCode == SUCCESSFUL_RETURN)
	{
	      real_t xOpt[T.getCols()];
cout << "we computed these forcev finally 1" << endl;
	      example.getPrimalSolution( xOpt );
cout << "we computed these forcev finally 2" << endl;
	printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e, %e, %e ]", 
				xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],xOpt[6],xOpt[7] );
for (int i = 0; i < T.getCols(); i++)
{
cout << "This is xOPt " << xOpt[i] << " This is the i " << i << endl;
}

cout << "we computed these forcev finally 3" <<endl;
forcev.resize(T.getCols(),true);
	for (int i = 0; i<(T.getCols()); i++)
	{ 
	forcev[i] = xOpt[i];
	}

	cout << "we computed these forcev finally" << forcev.t()<< endl;
	
	}else
{cout << "Could not successfully return good forces calculated  " << endl;
// We assign the minimum forces when we find no optimal solution
forcev = fmin; }
return forcev;
}




















}

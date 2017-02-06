#include <cdpr3/trajectory.h>

Trajectory::Trajectory(){}


Trajectory::~Trajectory()
{ 
	if( pcoefficients )
	{
		delete [] pcoefficients;
		pcoefficients = NULL;
	}
}


void Trajectory::init( double coefficients[], int nterms )
{
	Nterms = nterms;
	pcoefficients = new double[ Nterms ];
	for(int i=0; i<Nterms; i++)
		pcoefficients[i] = coefficients[i];
}

double Trajectory::horner(vector<double> v, double x)
{
  double s = 0;
 
  for( vector<double>::const_reverse_iterator i = v.rbegin(); i != v.rend(); i++ )
    s = s*x + *i;
  return s;
}

vector<double>  Trajectory::polynomial_derivative( const std::vector<double> & array )
{
  if ( 1 < array.size() )
  {
    std::vector<double> result( array.begin()+1, array.end()); 
    const size_t order = result.size();
    const size_t orderp = array.size();
 cout<< "The order is --- " << order << endl;
    for ( size_t i = 0; i < order; ++i )
    {
      result[ i ] =  (i + 1) * array[i+1];
    }
    return result;
  }
  
}


/*
vector<double> Trajectory::Trajectory5(double x,double xf,double xd,double xdf,double xdd,double xddf,double start_time,double end_time)
{
// Parameters(initial position, final position, initial velocity, final velocity, initial acceleration, final acceleration, start time , end time)

double  T = end_time - start_time;
double  a0 = x;
double	a1 = xd;
double	a2 = 0.5 * xdd;
double	a3 =(1.0/(2.0*T*T*T)) * ((20.0 * (xf - x)) - ((8.0 * xdf)+ (12.0*xd))*T - ((3.0 * xddf) - xdd )*(T*T));
double	a4 =(1.0/(2.0*T*T*T*T)) * ((30.0 * (x - xf)) + ((14.0 * xdf) + (16.0*xd))*T + ((3.0 * xddf) - (2.0*xdd) )*(T*T));
double	a5 =(1.0/(2.0*T*T*T*T*T)) * ((12.0 * (xf - x)) - 6.0*(xdf+ xd )*T - (xddf - xdd )*(T*T));
double uto[]  = {a0,a1,a2,a3,a4,a5}; 
vector<double> v(uto, uto + sizeof(uto) / sizeof(uto[5]) );

return v;
}
*/



vector<double> Trajectory::Trajectory5(double x,double xf,double xd,double xdf,double xdd,double xddf,double start_time,double end_time)
{

MatrixXd A(6,6);
VectorXd B_vec(6);
vector<double> result;


double t0 = start_time/1000;
 double tf = end_time/1000;
  // We do not use the pow() function due to precision issues for mm level trajectory planning

A << 1, t0, t0*t0, t0*t0*t0, t0*t0*t0*t0, t0*t0*t0*t0*t0,
     0, 1, 2*t0, 3*t0*t0, 4*t0*t0*t0, 5*t0*t0*t0*t0,
     0, 0, 2, 6*t0, 12*t0*t0, 20*t0*t0*t0,
     1, tf, tf*tf, tf*tf*tf, tf*tf*tf*tf, tf*tf*tf*tf*tf,
     0, 1, 2*tf, 3*tf*tf, 4*tf*tf*tf, 5*tf*tf*tf*tf,
     0, 0, 2, 6*tf, 12*tf*tf, 20*tf*tf*tf;
 std::cout << A;

 B_vec << x,xd,xdd,xf,xdf,xddf;
 std::cout << B_vec;

 VectorXd  coefficients = A.fullPivHouseholderQr().solve(B_vec);

 std::cout <<  coefficients ;

for(int i = 0;i <coefficients.size(); i++)
{
result.push_back(coefficients(i));
}
 return result;
}

/*
vector<double> Trajectory::Trajectory5(double x,double xf,double xd,double xdf,double xdd,double xddf,double start_time,double end_time)
{
vpMatrix A = vpMatrix(6,6);
vpColVector B_vec =  vpColVector(6); 
A.matlabPrint(std::cout);
cout << "This is B " << B_vec.t() << endl;
vector<double> result;

  // We do not use the pow() function due to precision issues for mm level trajectory planning
  A[0][0] = 1.0; A[0][1] = start_time; A[0][2] = start_time*start_time; A[0][3] = start_time*start_time*start_time; A[0][4] =start_time*start_time*start_time*start_time; A[0][5] = start_time*start_time*start_time*start_time*start_time; 
  A[1][0] = 0.0; A[1][1] = 1.0; A[1][2] = 2.0*start_time; A[1][3] = 3.0*start_time*start_time; A[1][4] = 4.0*start_time*start_time*start_time; A[1][5] = 5.0*start_time*start_time*start_time*start_time;
  A[2][0] = 0.0; A[2][1] = 0.0; A[2][2] = 2.0; A[2][3] = 6.0*start_time; A[2][4] = 12.0*start_time*start_time; A[2][5] = 20.0*start_time*start_time*start_time; 
  A[3][0] =  1.0; A[3][1] =  end_time; A[3][2] = end_time*end_time; A[3][3] = end_time*end_time*end_time; A[3][4] = end_time*end_time*end_time*end_time; A[3][5] = end_time*end_time*end_time*end_time*end_time; 
  A[4][0] = 0.0; A[4][1] = 1.0; A[4][2] = 2.0*end_time; A[4][3] = 3.0*end_time*end_time; A[4][4] =  4.0*end_time*end_time*end_time; A[4][5] = 5.0*end_time*end_time*end_time*end_time; 
  A[5][0] = 0.0; A[5][1] = 0.0; A[5][2] = 2.0; A[5][3] = 6.0*end_time; A[5][4] = 12.0*end_time*end_time; A[5][5] = 20.0*end_time*end_time*end_time; 


A.matlabPrint(std::cout);
B_vec[0] = x;
B_vec[1] = xd;
B_vec[2] = xdd;
B_vec[3] = xf;
B_vec[3] = xdf;
B_vec[5] = xddf;

vpColVector coefficients = A.inverseByQRLapack()*B_vec;

for(int i = 0;i <coefficients.size(); i++)
{
result.push_back(coefficients[i]);
}
 return result;
}
*/

vector<double> Trajectory::linspace(double min, double max, int n)
{
 vector<double> result;
 int iterator = 0;
 
for (int i = 0; i <= n-2; i++)
 {
 double temp = min + i*(max-min)/(floor((double)n) - 1);
 result.insert(result.begin() + iterator, temp);
 iterator += 1;
 }
 result.insert(result.begin() + iterator, max);
 return result;
}

double Trajectory::getposition5(vector<double> v, double x)
{
x = x/1000.0;
double position = v[0] + v[1]*x + (v[2]*x*x) + (v[3]*x*x*x) + (v[4]*x*x*x*x) + (v[5]*x*x*x*x*x);
return position;  
}

double Trajectory::getvelocity5(vector<double> v, double x)
{
x = x/1000.0;
double velocity = v[1] + (2.0*v[2]*x) + (3.0*v[3]*x*x) + (4.0*v[4]*x*x*x) + (5.0*v[5]*x*x*x*x);
return velocity/1000.0;  
}


double Trajectory::getacceleration5(vector<double> v, double x)
{
x = x/1000.0;
double acceleration = (2.0*v[2]) + (6.0*v[3]*x) + (12.0*v[4]*x*x) + (20.0*v[5]*x*x*x);
return acceleration/1000000.0; 
}



void Trajectory::print(void)
{
	
	std::cout << pcoefficients[Nterms-1] << "x^" << Nterms-1;
	for(int i=Nterms-2; i>=0; i--)			
		std::cout << " + " << pcoefficients[i] << "x^" << i;		
	return;
}


void Trajectory::ApplyForce(vpColVector effort, vector<string> joint_names)
{
for(unsigned int i=0;i<joint_names.size();++i)
            {          
         gazebo_msgs::ApplyJointEffort srv;
         srv.request.joint_name = joint_names[i];
	 srv.request.effort = effort[i];
         srv.request.duration = ros::Duration(0.1);
         srv.request.start_time = ros::Time::now();
         if(ros::service::exists("/gazebo/apply_joint_effort", true))
       {                   
          ros::service::call("/gazebo/apply_joint_effort", srv) ;
 	  cout << "Applying the wrench  " <<effort[i] << "To prismatic joint "<< joint_names[i] << endl; 
          ROS_INFO("Service ApplyJointEffort called successfully ---- BRILLIANT!!!");
        }
else
{
ROS_INFO("Service ApplyJointEffort not called successfully ---- Problems?!!!");
}
        
                
            
  
}

}

vector<vpTranslationVector> Trajectory::GetLink(vector<string> link_name)
{
for(unsigned int i=0;i<link_name.size();++i)
            {
         gazebo_msgs::GetLinkState srv;
         srv.request.link_name = link_name[i];
	 if(ros::service::exists("/gazebo/get_link_state", true))
       {                   
          ros::service::call("/gazebo/get_link_state", srv) ;
 	  //cout << "Getting the link position of " << link_name[i]<< "...... " <<endl; 
	  //cout <<"It is" << endl;
 	  //std::cout << srv.response.link_state.pose.position.x << std::endl;
  	  //std::cout << srv.response.link_state.pose.position.y << std::endl;
  	  //std::cout << srv.response.link_state.pose.position.z << std::endl; 
          //ROS_INFO("Service GetLink Stat called successfully ---- BRILLIANT!!!");
          vector<vpTranslationVector> pose;
          pose.push_back(vpTranslationVector(srv.response.link_state.pose.position.x, srv.response.link_state.pose.position.y, srv.response.link_state.pose.position.z));
          return pose;
        }
else
{
ROS_INFO("Service  GetLink State not called successfully ---- Problems?!!!");
}
        
                
            
  
}

}


































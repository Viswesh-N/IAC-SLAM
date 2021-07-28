#include<indy_slam/indyOdom.hpp>																							
#include<cmath>

namespace indySLAM {					

indyOdom::indyOdom(ros::NodeHandle& nodeHandle) : nh_(nodeHandle)																					
{	
	if (!readParams()) {																									//this block of code calls the readParams() function and if the return value is false then the code is shut down
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	sub_ = nh_.subscribe("/carla/vehicle/086/odometry", 10, &indyOdom::callback, this);										//the data member "sub_" of the class indyOdom is being made to subscribe to the specified topic and call the callback function of the class
	output_pub  = nh_.advertise<nav_msgs::Odometry>("noisy_odom", 10);														//the two data members output_pub and input_pub are set up to publish to the respective topics
	input_pub 	= nh_.advertise<nav_msgs::Odometry>("clean_odom", 10);

	ROS_INFO("Node Successfully Launched");																					//printing
}

indyOdom::~indyOdom()																										//destructor to deinitiaize the values 
{
}
void indyOdom::gaussian_noise(const nav_msgs::Odometry::ConstPtr& msg)														//definition of the function which adds noise
{
	clean_data = noisy_data = *msg;																							//passing the input as it is to the clean and noisy variables

	std::random_device rd;																									//initializing the randomizing variable
	std::default_random_engine generator;																					//using a generator variable
	std::normal_distribution<double> linear_distribution(0, position_sd), angular_distribution(0, M_PI*orientation_sd/180);	//this is used for setting the mean and standard deviation for the noisy output

	generator.seed(rd()); 																									//resets the random data that is generated so that the same set of random data is not generated each time the block of code runs

	noisy_data.pose.pose.position.x += linear_distribution(generator);														//the following 6 lines add the generated error to the parameters
	noisy_data.pose.pose.position.y += linear_distribution(generator);
	noisy_data.pose.pose.position.z += linear_distribution(generator);

	noisy_data.pose.pose.orientation.x += angular_distribution(generator);
	noisy_data.pose.pose.orientation.y += angular_distribution(generator);
	noisy_data.pose.pose.orientation.z += angular_distribution(generator);

	ROS_INFO("Data Succesfully Added");																						//printing 
}


bool indyOdom::readParams()
{																															//this function is responsible for reading data from the .yaml file;
	bool success = true;	
	success &= nh_.getParam("/indyOdom/position_std_deviation", position_sd);												//position_std_deviation and orientation_std_deviation have been added in the parameter file.
	success &= nh_.getParam("/indyOdom/orientation_std_deviation", orientation_sd);											//these are then being written into position_sd and orientation_sd; if one needs to change the values, the changes can be made on the .yaml file in the include directory of the package
	return success;																											//the boolean value is returned after performing two boolean "&" operations to ensure the values have been copied correctly
}

void indyOdom::callback(const nav_msgs::Odometry::ConstPtr& msg)															//the main callback function; this is the function that is called when the subsciber receives data
{
	indyOdom::gaussian_noise(msg);																							//the computational function is being called; this is used for 
	indyOdom::clean_data_publish();																							//calls the member functions of the class responsible for publishing the clean and noise-added data
	indyOdom::noisy_data_publish();																						
}



void indyOdom::noisy_data_publish()																							//member function definition for publishing the noisy data
{
	output_pub.publish(noisy_data);																							//publishing the noisy data
}

void indyOdom::clean_data_publish()																							//member funtioin definition for publishing clean data
{	
	input_pub.publish(clean_data);																							//publishing the clean data
}

}	/* namespace */
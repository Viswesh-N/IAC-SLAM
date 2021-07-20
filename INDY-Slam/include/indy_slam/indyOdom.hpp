#pragma once                                                                    
#include <ros/ros.h>                                                              
#include <nav_msgs/Odometry.h>                                                  
#include <random>

namespace indySLAM {                                                            

class indyOdom                                                                 
{

public:                                                                         
	indyOdom(ros::NodeHandle& nodeHandle);
	virtual ~indyOdom();

private:                                                                        
	
	void callback(const nav_msgs::Odometry::ConstPtr& msg);                     
	
	void gaussian_noise(const nav_msgs::Odometry::ConstPtr &msg);               
	
	void noisy_data_publish();                                                  
	void clean_data_publish();                                                  
	bool readParams();

	ros::NodeHandle &nh_;                                                       
	
	ros::Subscriber sub_;

	ros::Publisher output_pub;                                                  
	ros::Publisher input_pub;

	
	nav_msgs::Odometry noisy_data;                                              
	nav_msgs::Odometry clean_data;                                              
	

	float position_sd;                                                          
	float orientation_sd;
	

};                                                                              

}                                                                               

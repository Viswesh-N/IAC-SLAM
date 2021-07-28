#include <ros/ros.h>                                                    
#include "indy_slam/indyOdom.hpp"       		                        

int main(int argc, char** argv)
{
	ros::init(argc, argv, "noise_generator");                           //initializing the node with the name "noise_generator"
	ros::NodeHandle nodeHandle("~");                                    //initializng the node handle

	indySLAM::indyOdom example(nodeHandle);								//creating an object 'example' and initializing it with the node handle;

	ros::spin();
	return 0;
}
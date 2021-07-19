#include <cmath>														// including the necessary headers: random is a header for generating 
#include <ros/ros.h>													// the gaussian noise
#include <random>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry noisymsg;												//declaring a global variable of type Odometry under the namespace nav_msgs
ros::Publisher odom_noisy;												//publisher that will publish the noisy data
ros::Publisher odom_clean;												//publisher that will publish the clean data, is included just for comparison purposes

void Callback(const nav_msgs::Odometry::ConstPtr &input)				//defining a callback function which takes in a pointer to an odometry message as its argument
{
	std::random_device rd;												//adding the randomizing variable which will generate the noise
	std::default_random_engine generator;								//these two blocks of code add the error to the variables
        generator.seed( rd() ); 										//this line is used to generate a new set of errors each time the code is executed
        std::normal_distribution<double> distribution(0,0.06);			//mean=0, SD=0.06 for the position variables 
		                                                        	    //mean=0, SD= 4 degrees (4*pi/180) for orientation variables
	std::default_random_engine generator1;								//the arguments inside distribution() correspond to the mean and standard deviation respectively
        generator1.seed( rd() );
        std::normal_distribution<double> distribution1(0,4*(M_PI)/180);

	noisymsg.pose.pose.position = input->pose.pose.position;			//the x,y,z variables in "position" of the input message is written to noisymsg first
	noisymsg.pose.pose.position.x += distribution(generator);			//adding the noise that was generated to each of the x,y,z coordinates
	noisymsg.pose.pose.position.y += distribution(generator);			
	noisymsg.pose.pose.position.z += distribution(generator);

	noisymsg.pose.pose.orientation = input->pose.pose.orientation;		//the x,y,z variables in "position" of the input message is written to noisymsg first
	noisymsg.pose.pose.orientation.x += distribution(generator);		//adding the noise that was generated to each of the x,y,z coordinates
	noisymsg.pose.pose.orientation.y += distribution(generator);			
	noisymsg.pose.pose.orientation.z += distribution(generator);

	noisymsg.header = input->header;									//assigning the remaining parameters of the input to the noisy output without any change
	noisymsg.twist = input->twist;
	noisymsg.child_frame_id =  input->child_frame_id;

	odom_noisy.publish(noisymsg);										//publishing the noisy message
	odom_clean.publish(*input);											//publishing the clean message(* used since input is a pointer)

	
}

int main(int argc, char** argv){																			
 	ros::init(argc, argv, "noise_generator");
	ros::NodeHandle nh;																						//initializing the node handle
	odom_noisy = nh.advertise<nav_msgs::Odometry>("/noisy_odom",100);										//setting up the publishers to publish to diff topics 
	odom_clean = nh.advertise<nav_msgs::Odometry>("/clean_odom",100);
	ros::Subscriber odomSubscriber = nh.subscribe("/carla/vehicle/086/odometry", 1000, Callback);			//setting up a subscriber to subscribe to the topic where the Odometry message is being published by the bag file and call the callback function
	ros::spin();
 	return 0;
}



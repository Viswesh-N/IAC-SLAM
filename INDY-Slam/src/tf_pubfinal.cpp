#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/point_types.hpp>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

pcl::PointCloud<pcl::PointXYZ> final_cloud;
ros::Publisher tf_publisher;

int i=0;
void tf_callback(const tf2_msgs::TFMessage::ConstPtr& tf){
	tf2_msgs::TFMessage tf_ = *tf;
	
	if(sizeof(tf_.transforms)/sizeof(tf_.transforms[0])>1){
		tf_publisher.publish(tf_.transforms[1]);
	}
	else{
		tf_publisher.publish(tf_.transforms[0]);
	}
	i++;
	ROS_INFO_STREAM("Tf_data published "<< i);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_pub");
	ros::NodeHandle nh;
	
	
	tf_publisher = nh.advertise<geometry_msgs::TransformStamped>("/tf_data",100);
	ros::Subscriber tf_subscriber = nh.subscribe("/tf", 100, tf_callback);

	ros::spin();
	return 0;
}



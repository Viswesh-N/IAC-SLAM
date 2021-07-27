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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
int j=0;
sensor_msgs::PointCloud2 output;
ros::Publisher local_map_publisher;
void localisation_callback(const nav_msgs::Odometry::ConstPtr& odom){
	pcl::PointXYZ searchPoint;
	pcl::PointXYZ temp;
	searchPoint.x = (odom->pose).pose.position.x;
	searchPoint.y = (odom->pose).pose.position.y;
	searchPoint.z = (odom->pose).pose.position.z;
	pcl::PointCloud<pcl::PointXYZ> local_map;
	std::vector<int> pointIdxRadiusSearch;
  	std::vector<float> pointRadiusSquaredDistance;
	local_map.width = 0;
	local_map.height = 1;
	local_map.points.clear();
	kdtree.setInputCloud(cloud);
	float radius = 15.0;
	if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
    	for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
			temp.x = (*cloud)[ pointIdxRadiusSearch[i] ].x;
			temp.y = (*cloud)[ pointIdxRadiusSearch[i] ].y;
			temp.z = (*cloud)[ pointIdxRadiusSearch[i] ].z; 

			local_map.points.push_back(temp);
			local_map.width++;
			ROS_INFO_STREAM("Point added to local map: "<<temp.x<<" "<<temp.y<<" "<<temp.z);
			
		}
		pcl::toROSMsg(local_map, output);
	 output.header.frame_id="map";
  	}
	ROS_INFO_STREAM("Outside if statement");
	local_map_publisher.publish(output);


}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "localisation");
	ros::NodeHandle nh;
	local_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/local_map",1);
  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/viswesh-n/Downloads/Merged2.pcd", *cloud) == -1) 
  	{
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return (-1);
  	}
	else{
		ROS_INFO_STREAM("PCD file read successfully");
	}
	ros::Subscriber noisy_odom_sum = nh.subscribe("IndyOdom/noisy_odom",1, localisation_callback);
	ros::spin();
	return 0;
}

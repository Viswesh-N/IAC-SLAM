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
#include <pcl/octree/octree_search.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(128.0f);
sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);
ros::Publisher local;
ros::Publisher lidar;



void icp_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl,const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::TransformStamped::ConstPtr& tf){
	lidar.publish(*pcl);
	
	ROS_INFO_STREAM("Callback called");
    sensor_msgs::PointCloud2::Ptr pcl_cloud(new sensor_msgs::PointCloud2);
	pcl_ros::transformPointCloud("map", tf->transform, *pcl, *pcl_cloud);
    
    
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
	
	float radius = 30.0;
	if ( octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
		int n=0;
    	for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
			temp.x = (*cloud)[ pointIdxRadiusSearch[i] ].x;
			temp.y = (*cloud)[ pointIdxRadiusSearch[i] ].y;
			temp.z = (*cloud)[ pointIdxRadiusSearch[i] ].z; 
			n++;
			local_map.points.push_back(temp);
			local_map.width++;
			//ROS_INFO_STREAM("Point added to local map: "<<temp.x<<" "<<temp.y<<" "<<temp.z);
		}
		//ROS_INFO_STREAM(n);
  	}
	//ROS_INFO_STREAM("Outside if statement");
	pcl::toROSMsg(local_map, *output_cloud);
	(output_cloud->header).frame_id="map";
	local.publish(*output_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_scan (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pcl_cloud, *lidar_scan);

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*output_cloud, *local_map_);

   

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource (local_map_);
    icp.setInputTarget (lidar_scan);
    //icp.setMaxCorrespondenceDistance (0.5);
    //icp.setTransformationEpsilon (1e-3);
    //icp.setEuclideanFitnessEpsilon(1);
    icp.setMaximumIterations(5);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    ROS_INFO_STREAM(transformation);
    ROS_INFO_STREAM("################################");	
}


int main(int argc, char** argv)
{
	

  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/viswesh-n/catkin_ws/src/final_pcd_down.pcd", *cloud) == -1) //* load the file
  	{
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return (-1);
  	}
	else{
		ROS_INFO_STREAM("PCD file read successfully");
		octree.setInputCloud (cloud);
  		octree.addPointsFromInputCloud ();
	}

	ros::init(argc, argv, "icp");
	ros::NodeHandle nh;
	lidar = nh.advertise<sensor_msgs::PointCloud2>("/lidar",100);
	local = nh.advertise<sensor_msgs::PointCloud2>("/local",100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_subscriber(nh, "/carla/vehicle/086/lidar/front/point_cloud", 1);
	message_filters::Subscriber<nav_msgs::Odometry> noisy_odom_subscriber(nh, "/IndyOdom/noisy_odom", 1);
    message_filters::Subscriber<geometry_msgs::TransformStamped> tf_subscriber(nh, "/pub", 1);
	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry, geometry_msgs::TransformStamped> sync(pcl_subscriber, noisy_odom_subscriber, tf_subscriber, 1);
	sync.registerCallback(boost::bind(&icp_callback, _1, _2, _3));
	ros::spin();

	return 0;
}
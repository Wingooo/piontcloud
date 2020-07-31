#pragma once
#include <pcl/ModelCoefficients.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Float64MultiArray.h>
//点云下采样
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
class PclTestCore
{
 
  private:
    ros::Subscriber sub_point_cloud_;
 
    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_filtered_vis_;
    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
    void callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud,const sensor_msgs::NavSatFixConstPtr& ori_imu);
    void point_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const    pcl::PointCloud<pcl::PointXYZ>::Ptr out);
  public:
    PclTestCore(ros::NodeHandle &nh);   //构造函数
    ~PclTestCore();                     //析构函数
    void Spin(); 
};


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>

#include "tools.hpp"

// ROS related
ros::Publisher pub_path_orig, pub_path_opt, pub_map_orig, pub_map_opt, pub_plane;
ros::Subscriber sub_odom, sub_cloud;
ros::Timer exec_timer;

// Storage
IMUST latest_pose;
IMUST key_pose;
pcl::PointCloud<PointType>::Ptr key_pcl;
std::vector<IMUST> poses_vec;
std::vector<IMUST> poses_vec_opt;
std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec;
std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec_opt;

int start_index = 0;
int input_counter = 0;
int slide_size;
int keyframe_size;
std::mutex mtx;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
void timer_callback(const ros::TimerEvent &event);

void publish_pointcloud(const pcl::PointCloud<PointType> &cloud, const ros::Publisher &pub);

void visualize(const vector<IMUST> &poses, const vector<pcl::PointCloud<PointType>::Ptr> &pcls, const ros::Publisher &pub_pos, const ros::Publisher &pub_pcl);

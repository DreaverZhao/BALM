#include <string>
#include <vector>
#include <mutex>
#include <signal.h>
#include <malloc.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "tools.hpp"
#include "bavoxel.hpp"
#include "run_real.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BALM");
    ros::NodeHandle n;

    pub_path_orig = n.advertise<sensor_msgs::PointCloud2>("/path_orig", 100);
    pub_path_opt = n.advertise<sensor_msgs::PointCloud2>("/path_opt", 100);
    pub_map_orig = n.advertise<sensor_msgs::PointCloud2>("/map_orig", 100);
    pub_map_opt = n.advertise<sensor_msgs::PointCloud2>("/map_opt", 100);
    pub_plane = n.advertise<sensor_msgs::PointCloud2>("/plane", 100);

    sub_odom = n.subscribe("/Odometry", 100, odom_callback);
    sub_cloud = n.subscribe("/cloud_registered_body", 100, cloud_callback);

    exec_timer = n.createTimer(ros::Duration(0.3), timer_callback);

    n.param<double>("voxel_size", voxel_size, 2);
    n.param<int>("min_plane_size", min_plane_size, 20);
    n.param<int>("keyframe_size", keyframe_size, 5);
    n.param<int>("window_size", win_size, 20);
    n.param<int>("slide_size", slide_size, 10);

    exec_timer.start();
    ros::spin();

    return 0;
}

void timer_callback(const ros::TimerEvent &event)
{
    std::vector<IMUST> poses_vec_copy;
    std::vector<pcl::PointCloud<PointType>::Ptr> pcl_vec_copy;
    mtx.lock();

    if (poses_vec_opt.size() < start_index + win_size) // not enough data
    {
        mtx.unlock();
        return;
    }

    exec_timer.stop(); // Avoid re-entry

    // Copy the data
    for (int i = start_index; i < poses_vec_opt.size(); i++)
    {
        poses_vec_copy.push_back(poses_vec_opt[i]);
        pcl_vec_copy.push_back(pcl_vec_opt[i]);
    }
    visualize(poses_vec, pcl_vec, pub_path_orig, pub_map_orig);

    ROS_INFO("Optimizing from frame: %d", start_index);
    mtx.unlock();

    // Normalize the data
    IMUST es0 = poses_vec_copy[0];
    for (uint i = 0; i < poses_vec_copy.size(); i++)
    {
        poses_vec_copy[i].p = es0.R.transpose() * (poses_vec_copy[i].p - es0.p);
        poses_vec_copy[i].R = es0.R.transpose() * poses_vec_copy[i].R;
    }

    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> surf_map;

    eigen_value_array[0] = 1.0 / 16;
    eigen_value_array[1] = 1.0 / 16;
    eigen_value_array[2] = 1.0 / 9;

    for (int i = 0; i < win_size; i++)
        cut_voxel(surf_map, *pcl_vec_copy[i], poses_vec_copy[i], i);

    pcl::PointCloud<PointType> pcl_plane;
    VOX_HESS voxhess;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
    {
        iter->second->recut(win_size);
        iter->second->tras_opt(voxhess, win_size);
        iter->second->tras_display(pcl_plane, win_size);
    }

    publish_pointcloud(pcl_plane, pub_plane);

    if (voxhess.plvec_voxels.size() < 3 * poses_vec_copy.size())
    {
        printf("Initial error too large.\n");
        printf("Please loose plane determination criteria for more planes.\n");
        printf("The optimization is skipped.\n");
        start_index += slide_size;
        exec_timer.start();
        return;
    }

    // Optimize the poses
    BALM2 opt_lsv;
    opt_lsv.damping_iter(poses_vec_copy, voxhess);

    for (auto iter = surf_map.begin(); iter != surf_map.end();)
    {
        delete iter->second;
        surf_map.erase(iter++);
    }
    surf_map.clear();
    malloc_trim(0);

    // Denormalize the data
    for (uint i = 0; i < poses_vec_copy.size(); i++)
    {
        poses_vec_copy[i].p = es0.R * poses_vec_copy[i].p + es0.p;
        poses_vec_copy[i].R = es0.R * poses_vec_copy[i].R;
    }

    mtx.lock();
    // Update the optimized poses
    for (int i = 0; i < poses_vec_copy.size(); i++)
    {
        poses_vec_opt[start_index + i] = poses_vec_copy[i];
        pcl_vec_opt[start_index + i] = pcl_vec_copy[i];
    }
    visualize(poses_vec_opt, pcl_vec_opt, pub_path_opt, pub_map_opt);
    start_index += slide_size;
    mtx.unlock();
    exec_timer.start();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    IMUST pose;
    pose.p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    pose.R = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
    pose.t = msg->header.stamp.toSec();
    mtx.lock();
    latest_pose = pose;
    mtx.unlock();
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *cloud);
    mtx.lock();
    if (!input_counter) // new key frame
    {
        key_pose = latest_pose;
        key_pcl = cloud;
        input_counter++;
    }
    else
    {
        IMUST es0 = key_pose;
        IMUST es1 = latest_pose;
        es1.R = es0.R.transpose() * es1.R;
        es1.p = es0.R.transpose() * (es1.p - es0.p);
        pl_transform(*cloud, es1);
        *key_pcl += *cloud;
        input_counter++;
        if (input_counter == keyframe_size) // {keyframe_size} frames as a window
        {
            poses_vec.push_back(key_pose);
            pcl_vec.push_back(key_pcl);
            poses_vec_opt.push_back(key_pose);
            pcl_vec_opt.push_back(key_pcl);
            input_counter = 0;
        }
    }
    mtx.unlock();
    return;
}

void publish_pointcloud(const pcl::PointCloud<PointType> &cloud, const ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

void visualize(const vector<IMUST> &poses, const vector<pcl::PointCloud<PointType>::Ptr> &pcls, const ros::Publisher &pub_pos, const ros::Publisher &pub_pcl)
{
    pcl::PointCloud<PointType> pl_send, pl_path;
    int winsize = poses.size();
    for (int i = 0; i < winsize; i++)
    {
        pcl::PointCloud<PointType> pl_tem = *pcls[i];
        down_sampling_voxel(pl_tem, 0.2);
        pl_transform(pl_tem, poses[i]);
        pl_send += pl_tem;

        // if ((i % 200 == 0 && i != 0) || i == winsize - 1)
        // {
        //     publish_pointcloud(pl_send, pub_pcl);
        //     pl_send.clear();
        //     sleep(0.1);
        // }

        PointType ap;
        ap.x = poses[i].p.x();
        ap.y = poses[i].p.y();
        ap.z = poses[i].p.z();
        ap.curvature = i;
        pl_path.push_back(ap);
    }

    publish_pointcloud(pl_send, pub_pcl);
    publish_pointcloud(pl_path, pub_pos);
}
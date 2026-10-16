#ifndef _FREENODE_H
#define _FREENODE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <thread>
#include <atomic>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <std_srvs/Trigger.h>

#include "freedom/freedom.h"
#include "freedom/visualization.h"

namespace freedom{
class FreeNode{
public:
    FreeNode();

    ~FreeNode();

private:
    std::unique_ptr<FreeDOM> static_map;
    FreeDOM::Config map_config{};
    void create_map();
    bool reset_map(std_srvs::Trigger::Request&,std_srvs::Trigger::Response&);
    ros::ServiceServer reset_service;
    ros::Subscriber state_sub,landed_sub;
    mavros_msgs::State state;
    mavros_msgs::ExtendedState landed;
    ros::Time state_received,landed_received,minimum_cloud_stamp;
    double cloud_max_age=.35;
    std::atomic<bool> stopping{false};

    std::string map_tf_frame;
    std::string sensor_tf_frame;

    ros::CallbackQueue callback_queue;
    std::thread thread;

    std::string pointcloud_topic;
    ros::Subscriber pointcloud_sub;

    std::string save_map_topic;
    std::string save_map_path;
    ros::Subscriber save_map_sub;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    void get_params(ros::NodeHandle& nh_private,FreeDOM::Config& map_config,Visualizer::Config& vis_config);
    void mapping_thread();
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
    void save_map_callback(const std_msgs::Empty::ConstPtr& msg);

    bool enable_visualization;
    bool enable_raycast_enhancement;
    Visualizer visualizer;

    Timer timer;
};
}
#endif

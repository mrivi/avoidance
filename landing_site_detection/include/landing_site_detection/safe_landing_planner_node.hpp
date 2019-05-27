#pragma once

#ifndef DISABLE_SIMULATION
// include simulation
#include "avoidance/rviz_world_loader.h"
#endif

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <landing_site_detection/SafeLandingPlannerNodeConfig.h>
#include <atomic>
#include <avoidance/common.h>

#include "safe_landing_planner.hpp"
#include "safe_landing_planner_visualization.hpp"


namespace avoidance {

  class SafeLandingPlannerNode {

  public:
    SafeLandingPlannerNode(const ros::NodeHandle& nh);
    ~SafeLandingPlannerNode() = default;

    std::unique_ptr<SafeLandingPlanner> landing_site_detection_;

#ifndef DISABLE_SIMULATION
    std::unique_ptr<avoidance::WorldVisualizer> world_visualizer_;
#endif

    std::atomic<bool> should_exit_{false};
    /**
    * @brief spins node
    **/
    void startNode();

  private:

    ros::Timer cmdloop_timer_;

    ros::NodeHandle nh_;

    ros::Publisher mavros_system_status_pub_;
    ros::Publisher grid_pub_;

    ros::Subscriber pose_sub_;
    ros::Subscriber pointcloud_sub_;

    tf::TransformListener tf_listener_;

    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped previous_pose_;
    mavros_msgs::CompanionProcessStatus status_msg_;

    ros::Time start_time_ = ros::Time(0.0);
    ros::Time last_algo_time_ = ros::Time(0.0);
    ros::Time t_status_sent_ = ros::Time(0.0);

    SafeLandingPlannerVisualization visualizer_;

    bool position_received_ = false;
    bool cloud_transformed_ = false;
    double spin_dt_ = 0.1;

    dynamic_reconfigure::Server<landing_site_detection::SafeLandingPlannerNodeConfig> server_;

    /**
    * @brief main loop callback
    * @param[in] event, event timing information
    **/
    void cmdLoopCallback(const ros::TimerEvent& event);

    /**
    * @brif callback for vehicle position and orientation
    * @param[in] msg, pose message coming fro the FCU
    **/
    void positionCallback(const geometry_msgs::PoseStamped& msg);
    /**
    * @brif callback for pointcloud
    * @param[in] msg, current frame pointcloud
    **/
    void pointCloudCallback(const sensor_msgs::PointCloud2& msg);
    /**
    * @brief     callaback for parameters dynamic reconfigure server
    * @param     config, struct with all the parameters
    * @param     level, bitmsak to group together reconfigurable parameters
    **/
    void dynamicReconfigureCallback(landing_site_detection::SafeLandingPlannerNodeConfig& config,
                                uint32_t level);

    /**
    * @brif     sends out a status to the FCU which will be received as a heartbeat
    **/
    void publishSystemStatus();

    /**
    * @brief      check healthiness of the avoidance system to trigger failsafe in the FCU
    * @param[in]  since_last_algo, time elapsed since the last iteration of the landing site detection algorithm
    * @param[in]  since_start, time elapsed since staring the node
    **/
    void checkFailsafe(ros::Duration since_last_algo, ros::Duration since_start);

    /**
    * @brief      publishes the computed grid fot the waypoint_generator_node to use
    **/
    void publishSerialGrid();

  };

}

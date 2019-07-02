#pragma once
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/WaypointList.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <safe_landing_planner/grid.hpp>

#include <safe_landing_planner/SLPGridMsg.h>

namespace avoidance {

class SafeLandingPlannerVisualizationNode {
 public:
  SafeLandingPlannerVisualizationNode(const ros::NodeHandle& nh);
  ~SafeLandingPlannerVisualizationNode() = default;
  void startNode();



private:
  Grid grid_slp_ = Grid(10.f, 1.f);
  ros::NodeHandle nh_;
  ros::Timer cmdloop_timer_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  ros::CallbackQueue cmdloop_queue_;

  ros::Subscriber pose_sub_;
  ros::Subscriber grid_sub_;
  ros::Publisher mean_pub_;
  ros::Publisher std_dev_pub_;
  ros::Publisher counter_pub_;
  void gridCallback(const safe_landing_planner::SLPGridMsg& msg);
  void cmdLoopCallback(const ros::TimerEvent& event);
  void positionCallback(
      const geometry_msgs::PoseStamped &msg);

  bool grid_received_ = false;
  double spin_dt_ = 0.1;
  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);

  void publishMean(const Grid& grid);
  void publishStandardDeviation(const Grid& grid);
  void publishCounter(const Grid& grid);
  std::tuple<float, float, float> HSVtoRGB(std::tuple<float, float, float> hsv);


};
}

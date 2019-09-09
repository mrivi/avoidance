#ifndef STAR_PLANNER_H
#define STAR_PLANNER_H

#include "avoidance/histogram.h"
#include "cost_parameters.h"
#include "trajectory_simulator.h"

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/GridCells.h>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <queue>
#include <vector>

namespace avoidance {

template <typename T, typename Container = std::deque<T>, typename Compare = std::less<typename Container::value_type> >
class iterable_priority_queue : public std::priority_queue<T, Container, Compare> {
 public:
  typedef typename Container::iterator iterator;
  typedef typename Container::const_iterator const_iterator;

  iterator begin() { return this->c.begin(); }
  iterator end() { return this->c.end(); }
  const_iterator begin() const { return this->c.begin(); }
  const_iterator end() const { return this->c.end(); }
};

class TreeNode;

class StarPlanner {
  int children_per_node_ = 8;
  int n_expanded_nodes_ = 40;
  float tree_node_duration_ = 0.5f;
  float max_path_length_ = 15.f;
  float smoothing_margin_degrees_ = 40.f;
  float tree_heuristic_weight_ = 35.f;
  float acceptance_radius_ = 2.f;
  float max_sensor_range_ = 15.f;
  float min_sensor_range_ = 0.2f;
  float tree_step_size_s_ = 0.05f;

  pcl::PointCloud<pcl::PointXYZI> cloud_;

  Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f velocity_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f closest_pt_ = Eigen::Vector3f(NAN, NAN, NAN);
  costParameters cost_params_;
  simulation_limits lims_;

 protected:
  /**
  * @brief     computes the heuristic for a node
  * @param[in] node_number, sequential number of entry in the tree
  * @returns
  **/
  float treeHeuristicFunction(const TreeNode &node) const;

 public:
  std::vector<Eigen::Vector3f> path_node_setpoints_;
  std::vector<int> closed_set_;
  std::vector<TreeNode> tree_;

  StarPlanner();
  ~StarPlanner() = default;

  /**
  * @brief     setter method for costMatrix paramters
  * @param[in] cost_params, parameters for the histogram cost function
  * @param[in] simulation limits defining maximum acceleration, velocity, and jerk
  **/
  void setParams(const costParameters& cost_params, const simulation_limits& limits, float acc_rad);

  /**
  * @brief     setter method for star_planner pointcloud
  * @param[in] cloud, processed data already cropped and combined with history
  **/
  void setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);

  /**
  * @brief     setter method for vehicle position
  * @param[in] vehicle current position
  **/
  void setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);

  /**
  * @brief     setter method for vehicle position projection on the line between the current and previous goal
  * @param[in] closest_pt, projection point
  **/
  void setClosestPointOnLine(const Eigen::Vector3f& closest_pt);

  /**
  * @brief     setter method for current goal
  * @param[in] goal, current goal position
  **/
  void setGoal(const Eigen::Vector3f& pose);

  /**
  * @brief     build tree of candidates directions towards the goal
  **/
  void buildLookAheadTree();

  /**
  * @brief     setter method for server paramters
  **/
  void dynamicReconfigureSetStarParams(const avoidance::LocalPlannerNodeConfig& config, uint32_t level);
};
}
#endif  // STAR_PLANNER_H

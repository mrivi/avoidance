#include "local_planner/star_planner.h"

#include "avoidance/common.h"
#include "local_planner/planner_functions.h"
#include "local_planner/tree_node.h"

#include <ros/console.h>

namespace avoidance {

StarPlanner::StarPlanner() {}

// set parameters changed by dynamic rconfigure
void StarPlanner::dynamicReconfigureSetStarParams(const avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  children_per_node_ = config.children_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_duration_ = static_cast<float>(config.tree_node_duration_);
  max_path_length_ = static_cast<float>(config.max_sensor_range_);
  smoothing_margin_degrees_ = static_cast<float>(config.smoothing_margin_degrees_);
  tree_heuristic_weight_ = static_cast<float>(config.tree_heuristic_weight_);
  max_sensor_range_ = static_cast<float>(config.max_sensor_range_);
  min_sensor_range_ = static_cast<float>(config.min_sensor_range_);
  tree_step_size_s_ = static_cast<float>(config.tree_step_size_s_);
}

void StarPlanner::setParams(const costParameters& cost_params, const simulation_limits& limits, float acc_rad) {
  cost_params_ = cost_params;
  lims_ = limits;
  acceptance_radius_ = acc_rad;
}

void StarPlanner::setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel) {
  position_ = pos;
  velocity_ = vel;
}

void StarPlanner::setGoal(const Eigen::Vector3f& goal) { goal_ = goal; }

void StarPlanner::setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) { cloud_ = cloud; }

void StarPlanner::setClosestPointOnLine(const Eigen::Vector3f& closest_pt) { closest_pt_ = closest_pt; }

float StarPlanner::treeHeuristicFunction(const TreeNode &node) const {
  return ((goal_ - node.getPosition()).norm() / lims_.max_xy_velocity_norm + node.state.time)* tree_heuristic_weight_;
}

void StarPlanner::buildLookAheadTree() {
  std::clock_t start_time = std::clock();
  Histogram histogram(ALPHA_RES);
  std::vector<uint8_t> cost_image_data;
  std::vector<candidateDirection> candidate_vector;
  Eigen::MatrixXf cost_matrix;
  Eigen::MatrixXf distance_matrix;
  Eigen::MatrixXf yaw_matrix;
  Eigen::MatrixXf yaw_line_matrix;
  Eigen::MatrixXf pitch_matrix;
  Eigen::MatrixXf velocity_matrix;

  bool is_expanded_node = true;

  tree_.clear();
  closed_set_.clear();

  // insert first node
  simulation_state start_state;
  start_state.position = position_;
  start_state.velocity = velocity_;
  start_state.acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  start_state.time = ros::Time::now().toSec();
  tree_.push_back(TreeNode(0, start_state, Eigen::Vector3f::Zero()));
  tree_.back().setCosts(treeHeuristicFunction(tree_.back()), treeHeuristicFunction(tree_.back()));

  int origin = 0;
  for (int n = 0; n < n_expanded_nodes_ && is_expanded_node; n++) {
    Eigen::Vector3f origin_position = tree_[origin].getPosition();
    Eigen::Vector3f origin_velocity = tree_[origin].getVelocity();
    PolarPoint facing_goal = cartesianToPolarHistogram(goal_, origin_position);
    float distance_to_goal = (goal_ - origin_position).norm();

    histogram.setZero();
    generateNewHistogram(histogram, cloud_, origin_position);

    // calculate candidates
    cost_matrix.fill(0.f);
    distance_matrix.fill(0.f);
    pitch_matrix.fill(0.f);
    yaw_matrix.fill(0.f);
    yaw_line_matrix.fill(0.f);
    velocity_matrix.fill(0.f);
    cost_image_data.clear();
    candidate_vector.clear();
    getCostMatrix(histogram, goal_, origin_position, origin_velocity, cost_params_, smoothing_margin_degrees_,
                  closest_pt_, max_sensor_range_, min_sensor_range_, cost_matrix, distance_matrix, velocity_matrix,
                  yaw_matrix, yaw_line_matrix, pitch_matrix, cost_image_data);

    simulation_limits limits = lims_;
    simulation_state state = tree_[origin].state;
    limits.max_xy_velocity_norm =
        std::min(computeMaxSpeedFromBrakingDistance(lims_.max_jerk_norm, lims_.max_acceleration_norm,
                                                    (tree_[origin].getPosition() - goal_).head<2>().norm()),
                 lims_.max_xy_velocity_norm);
    TrajectorySimulator sim(limits, state, tree_step_size_s_);
    iterable_priority_queue<candidateDirection, std::vector<candidateDirection>, std::less<candidateDirection>> queue;
    for (int row_index = 0; row_index < cost_matrix.rows(); row_index++) {
      for (int col_index = cost_matrix.cols() -1 ; col_index >= 0; col_index--) {
        PolarPoint p_pol = histogramIndexToPolar(row_index, col_index, ALPHA_RES, 1.0);
        float cost = cost_matrix(row_index, col_index);
        candidateDirection candidate(cost, p_pol.e, p_pol.z);
        candidate.setInvidualCosts(velocity_matrix(row_index, col_index), yaw_matrix(row_index, col_index),
      yaw_line_matrix(row_index, col_index), pitch_matrix(row_index, col_index), distance_matrix(row_index, col_index));
        simulation_state trajectory_endpoint =
            sim.generate_trajectory_endpoint(candidate.toEigen(), tree_node_duration_);
        int close_nodes = 0;
        for (auto it = queue.begin(); it != queue.end(); it++) {
          float dist = ((*it).tree_node.getPosition() - trajectory_endpoint.position).norm();
          if (dist < 0.1f) {
            close_nodes++;
            break;
          }
        }

        if (queue.size() < children_per_node_ && close_nodes == 0) {
          candidate.tree_node = TreeNode(origin, trajectory_endpoint, candidate.toEigen());
          queue.push(candidate);
        } else if (candidate < queue.top() && close_nodes == 0) {
          candidate.tree_node = TreeNode(origin, trajectory_endpoint, candidate.toEigen());
          queue.push(candidate);
          queue.pop();
        }
      }
    }

    int children = 0;
    while (!queue.empty()) {
      if (children < children_per_node_) {
        tree_.push_back(queue.top().tree_node);
        // v[queue.top().tree_node.origin_] = v[queue.top().tree_node.origin_] + 1;
        float h = treeHeuristicFunction(queue.top().tree_node);
        tree_.back().heuristic_ = h;
        tree_.back().total_cost_ = tree_[origin].total_cost_ - tree_[origin].heuristic_ + queue.top().cost + h;
        // printf("origin %d node pos %f %f %f cost %f h %f \n", queue.top().tree_node.origin_,
        // tree_.back().getPosition().x(), tree_.back().getPosition().y(), tree_.back().getPosition().z(),
        // tree_.back().total_cost_, h );
        // printf("costs dist %f, yaw %f, yaw line %f, pitch %f, velocity %f \n", queue.top().distance_cost,
        // queue.top().yaw_cost , queue.top().yaw_to_line_cost, queue.top().pitch_cost, queue.top().velocity_cost);
        queue.pop();
        children++;

      } else {
        break;
      }
    }

    closed_set_.push_back(origin);
    tree_[origin].closed_ = true;

    // find best node to continue
    float minimal_cost = HUGE_VAL;
    is_expanded_node = false;
    for (size_t i = 0; i < tree_.size(); i++) {
      if (!(tree_[i].closed_)) {
        // If we reach the acceptance radius, add goal as last node and exit
        if (i > 1 && (tree_[i].getPosition() - goal_).norm() < acceptance_radius_) {
          tree_.push_back(TreeNode(i, simulation_state(0.f, goal_), goal_ - tree_[i].getPosition()));
          closed_set_.push_back(i);
          closed_set_.push_back(tree_.size() - 1);
          break;
        }

        float node_distance = (tree_[i].getPosition() - position_).norm();
        // printf("cost %f < %f dist %f < %f \n", tree_[i].total_cost_, minimal_cost, node_distance, max_path_length_);
        if (tree_[i].total_cost_ < minimal_cost && node_distance < max_path_length_) {
          minimal_cost = tree_[i].total_cost_;
          origin = i;
        //   printf("min cost origin %d node %f %f %f \n", origin, tree_[i].getPosition().x(),
        // tree_[i].getPosition().y(), tree_[i].getPosition().z());
          is_expanded_node = true;
        }
      }
    }

    cost_image_data.clear();
    candidate_vector.clear();
  }
  int maximum_depth = -1;
  int chosen_children = 0;
  for (size_t i = 0; i < tree_.size(); i++) {
    size_t parent = tree_[i].origin_;
    int depth = 0;
    while(tree_[parent].origin_ > 0) {
      parent = tree_[parent].origin_;
      depth++;
    }

    if (maximum_depth < depth) {
      maximum_depth = depth;
      chosen_children = (int)i;
    }

  }

  printf("maximum_depth %d chosen_children %d \n", maximum_depth, chosen_children);

  // Get setpoints into member vector
  // std::vector<int>::iterator result = std::max_element(std::begin(v), std::end(v));
  // std::cout << "maxx element at: " << std::distance(std::begin(v), result) << " " << *std::max_element(std::begin(v), std::end(v)) << std::endl;;
  int tree_end = origin; //std::distance(std::begin(v), result);
  printf("\n\n treed end %d \n", origin );

  path_node_setpoints_.clear();
  while (tree_end > 0) {
    printf("(%f %f %f) ",tree_[tree_end].getPosition().x(), tree_[tree_end].getPosition().y(), tree_[tree_end].getPosition().z());
    path_node_setpoints_.push_back(tree_[tree_end].getSetpoint());
    tree_end = tree_[tree_end].origin_;
  }
  printf("--------------------------------\n");
  path_node_setpoints_.push_back(tree_[0].getSetpoint());
  std::cout << path_node_setpoints_.size() << std::endl;
}
}

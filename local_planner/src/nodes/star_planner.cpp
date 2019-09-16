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

float StarPlanner::treeHeuristicFunction(int node_number) const {
  return (goal_ - tree_[node_number].getPosition()).norm() * tree_heuristic_weight_;
}

void StarPlanner::buildLookAheadTree() {
  std::clock_t start_time = std::clock();

  Histogram histogram(ALPHA_RES);
  std::vector<uint8_t> cost_image_data;
  std::vector<candidateDirection> candidate_vector;
  Eigen::MatrixXf cost_matrix;

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
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));
  printf("*************************\n" );

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
    cost_image_data.clear();
    candidate_vector.clear();
    getCostMatrix(histogram, goal_, origin_position, origin_velocity, cost_params_, smoothing_margin_degrees_,
                  closest_pt_, max_sensor_range_, min_sensor_range_, cost_matrix, cost_image_data);
    if (n!=0) {
      starting_direction_ = Eigen::Vector3f(NAN, NAN, NAN);
    } else {
      printf("Previous starting direction %f %f %f \n", starting_direction_.x(), starting_direction_.y(), starting_direction_.z());
    }
    getBestCandidatesFromCostMatrix(cost_matrix, children_per_node_, candidate_vector, starting_direction_, position_);

    // add candidates as nodes
    if (candidate_vector.empty()) {
      tree_[origin].total_cost_ = HUGE_VAL;
    } else {
      // insert new nodes
      int children = 0;
      for (candidateDirection candidate : candidate_vector) {
        simulation_state state = tree_[origin].state;
        TrajectorySimulator sim(lims_, state, 0.05f);  // todo: parameterize simulation step size [s]
        std::vector<simulation_state> trajectory = sim.generate_trajectory(candidate.toEigen(), tree_node_duration_);

        // check if another close node has been added
        int close_nodes = 0;
        for (size_t i = 0; i < tree_.size(); i++) {
          float dist = (tree_[i].getPosition() - trajectory.back().position).norm();
          if (dist < 0.2f) {
            close_nodes++;
            break;
          }
        }

        // printf("node %f %f %f cost %f close %d %d \n", candidate.toEigen().x(), candidate.toEigen().y(), candidate.toEigen().z(),
        // candidate.cost, close_nodes, children < (children_per_node_));

        if (children < (children_per_node_) && close_nodes == 0) {
          tree_.push_back(TreeNode(origin, trajectory.back(), candidate.toEigen()));
          float h = treeHeuristicFunction(tree_.size() - 1);
          tree_.back().heuristic_ = h;
          tree_.back().total_cost_ = tree_[origin].total_cost_ - tree_[origin].heuristic_ + candidate.cost + h;

          if (n==0) {
            printf("AddTree origin %d node %f %f %f cost %f h %f \n", origin, candidate.toEigen().x(), candidate.toEigen().y(), candidate.toEigen().z(),
            tree_.back().total_cost_, h);
            Eigen::Vector2f candidate_dir = candidate.toEigen().head<2>();
            Eigen::Vector2f prev_init_dir_2f = starting_direction_.head<2>();
            float angle = 0.f;
            float add = costChangeInTreeDirection(prev_init_dir_2f, candidate_dir, angle);
            printf("angle diff %f cost %f \n", angle, add);
          }
          children++;
        }
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
        if (tree_[i].total_cost_ < minimal_cost && node_distance < max_path_length_) {
          minimal_cost = tree_[i].total_cost_;
          origin = i;
          is_expanded_node = true;
        }
      }
    }

    cost_image_data.clear();
    candidate_vector.clear();
  }

  float min_cost_per_depth = FLT_MAX;
  int chosen_children = 0;
  for (size_t i = 0; i < tree_.size(); i++) {
    if (tree_[i].closed_ == 0) {
      size_t parent = tree_[i].origin_;
      float total_cost = tree_[i].total_cost_;
      int depth = 0;
      while(tree_[parent].origin_ > 0) {
        parent = tree_[parent].origin_;
        depth++;
      }

      if (min_cost_per_depth > (total_cost / depth)) {
        min_cost_per_depth = (total_cost / depth);
        chosen_children = (int)i;
      }
    }
  }
  // Get setpoints into member vector
  int tree_end = chosen_children;
  path_node_setpoints_.clear();
  while (tree_end > 0) {
    path_node_setpoints_.push_back(tree_[tree_end].getSetpoint());
    // if (tree_[tree_end].origin_ == 0) {
    //   starting_direction_ = tree_[tree_end].getSetpoint();
    // }
    tree_end = tree_[tree_end].origin_;
  }


  path_node_setpoints_.push_back(tree_[0].getSetpoint());
  starting_direction_ = path_node_setpoints_[path_node_setpoints_.size() - 2];
  for (size_t i = 0; i < path_node_setpoints_.size(); i++) {
    printf("(%f %f %f) ", path_node_setpoints_[i].x(), path_node_setpoints_[i].y(), path_node_setpoints_[i].z());
  }
  printf("\n" );
}
}

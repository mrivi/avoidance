#include "landing_site_detection/waypoint_generator_node.hpp"
#include "landing_site_detection/landing_site_detection.hpp"
#include "avoidance/common.h"
#include "tf/transform_datatypes.h"

namespace avoidance {
  const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);


WaypointGeneratorNode::WaypointGeneratorNode(const ros::NodeHandle &nh) : nh_(nh), spin_dt_(0.1) {

  dynamic_reconfigure::Server<landing_site_detection::WaypointGeneratorNodeConfig>::CallbackType f;
  f = boost::bind(&WaypointGeneratorNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>("/mavros/local_position/pose", 1, &WaypointGeneratorNode::positionCallback, this);
  trajectory_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &WaypointGeneratorNode::trajectoryCallback, this);
  mission_sub_ = nh_.subscribe("/mavros/mission/waypoints", 1, &WaypointGeneratorNode::missionCallback, this);
  state_sub_ = nh_.subscribe("/mavros/state", 1, &WaypointGeneratorNode::stateCallback, this);
  grid_sub_ = nh_.subscribe("/grid_lsd", 1, &WaypointGeneratorNode::gridCallback, this);

  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  land_hysteresis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/land_hysteresis", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_position", 1);


}

void WaypointGeneratorNode::startNode() {
  cmdloop_timer_ = nh_.createTimer(ros::Duration(spin_dt_), boost::bind(&WaypointGeneratorNode::cmdLoopCallback, this, _1));
}


void WaypointGeneratorNode::cmdLoopCallback(const ros::TimerEvent& event) {

  ros::Time start_query_position = ros::Time::now();
  while (!grid_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  calculateWaypoint();
  landingAreaVisualization();
  goalVisualization();
  grid_received_ = false;

  return;

}

void WaypointGeneratorNode::dynamicReconfigureCallback(landing_site_detection::WaypointGeneratorNodeConfig& config, uint32_t level) {
  update_smoothing_size_ = false;
  beta_ = static_cast<float>(config.beta);
  landing_radius_ = static_cast<float>(config.landing_radius);
  can_land_thr_ = static_cast<float>(config.can_land_thr);
  loiter_height_ = static_cast<float>(config.loiter_height);
  smoothing_land_cell_ = config.smoothing_land_cell;

  if (can_land_hysteresis_.size() != ((smoothing_land_cell_ * 2) * (smoothing_land_cell_ * 2))) {
    update_smoothing_size_ = true;
  }

}

void WaypointGeneratorNode::positionCallback(const geometry_msgs::PoseStamped &msg) {
  position_ = avoidance::toEigen(msg.pose.position);
  double roll, pitch, yaw;

  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);
  yaw_ = static_cast<float>(yaw);
  ROS_INFO("[WGN] Current position %f %f %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

}


void WaypointGeneratorNode::trajectoryCallback(const mavros_msgs::Trajectory& msg) {

  bool update = (avoidance::toEigen(msg.point_2.position) - goal_visualization_).norm()> 0.01;

  if (update && msg.point_valid[0] == true) {
    goal_ = avoidance::toEigen(msg.point_1.position);
    ROS_INFO("\033[1;33m [WGN] Set New goal from FCU %f %f %f - nan nan nan \033[0m", goal_.x(), goal_.y(), goal_.z());
    velocity_setpoint_ = avoidance::toEigen(msg.point_1.velocity);
  }
  if (msg.point_valid[1] == true) {
    goal_visualization_ = avoidance::toEigen(msg.point_2.position);
    yaw_setpoint_ = msg.point_2.yaw;
    yaw_speed_setpoint_ = msg.point_2.yaw_rate;
  }

}

void WaypointGeneratorNode::missionCallback(const mavros_msgs::WaypointList& msg) {
  is_land_waypoint_ = false;
  for (auto waypoint : msg.waypoints) {
    if (waypoint.is_current && waypoint.command == 21) {
      is_land_waypoint_ = true;
    }
  }
}

void WaypointGeneratorNode::stateCallback(const mavros_msgs::State &msg) {

  if (msg.mode == "AUTO.TAKEOFF") {
    lsd_state_ = LSDState::goTo;
    is_land_waypoint_ = false;
  } else if (msg.mode == "AUTO.LAND") {
    is_land_waypoint_ = true;
  } else if (msg.mode == "AUTO.MISSION") {
    // is_land_waypoint_ is set trought the mission item type
  } else {
    is_land_waypoint_ = false;
    lsd_state_ = LSDState::goTo;
  }

  if (msg.armed == false) {
    is_land_waypoint_ = false;
  }

}

void WaypointGeneratorNode::gridCallback(const landing_site_detection::LSDGridMsg &msg) {
  grid_lsd_seq_ = msg.header.seq;
  if (grid_lsd_.grid_size_ != msg.grid_size || grid_lsd_.cell_size_ != msg.cell_size) {
    grid_lsd_.grid_size_ = msg.grid_size;
    grid_lsd_.cell_size_ = msg.cell_size;
    grid_lsd_.resize();
  }

  for (int i = 0; i < msg.mean.layout.dim[0].size; i++) {
    for (int j = 0; j < msg.mean.layout.dim[1].size; j++) {
      grid_lsd_.mean_(i, j) = msg.mean.data[grid_lsd_.mean_.cols() * i + j];
      grid_lsd_.land_(i, j) = msg.land.data[grid_lsd_.land_.cols() * i + j];
    }
  }

  pos_index_.x() = static_cast<int>(msg.curr_pos_index.x);
  pos_index_.y() = static_cast<int>(msg.curr_pos_index.y);

  grid_lsd_.setFilterLimits(position_);
  grid_received_ = true;

}

void WaypointGeneratorNode::calculateWaypoint() {

  updateLSDState();

  switch (lsd_state_) {
    case LSDState::goTo: {

      decision_taken_ = false;
      if (explorarion_is_active_) {
        landing_radius_ = 0.5f;
        yaw_setpoint_ = avoidance::nextYaw(position_, goal_);
      }
      publishTrajectorySetpoints(goal_, velocity_setpoint_, yaw_setpoint_, yaw_speed_setpoint_);
      ROS_INFO("\033[1;32m [WGN] goTo %f %f %f - %f %f %f \033[0m\n", goal_.x(), goal_.y(), goal_.z(), velocity_setpoint_.x(), velocity_setpoint_.y(), velocity_setpoint_.z());

      is_within_landing_radius_ = (goal_.topRows<2>() - position_.topRows<2>()).norm() < landing_radius_;
      in_land_vertical_range_ = fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())) < (loiter_height_ + 0.5f) &&
        fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())) > (loiter_height_ - 0.5f);
      ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ", (goal_.topRows<2>() - position_.topRows<2>()).norm(), fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())));

      if (is_within_landing_radius_ && !in_land_vertical_range_ && is_land_waypoint_ && !std::isfinite(velocity_setpoint_.z())) {
        prev_lsd_state_ = LSDState::goTo;
        lsd_state_ = LSDState::altitudeChange;
        ROS_INFO("\033[1;35m [WGN] Update to altitudeChange State \033[0m");
      }

      if (is_within_landing_radius_ && in_land_vertical_range_ && is_land_waypoint_) {
        start_seq_landing_decision_ = grid_lsd_seq_;
        prev_lsd_state_ = LSDState::goTo;
        lsd_state_ = LSDState::loiter;
        ROS_INFO("\033[1;34m [WGN] Update to Loiter State \033[0m");
      }
      break;
    }

    case LSDState::altitudeChange: {

      if (prev_lsd_state_ != LSDState::altitudeChange) {
        yaw_setpoint_ = yaw_;
      }
      goal_.z() = NAN;
      float direction = fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())) <= (loiter_height_ - 0.5f) ? 1.f : -1.f;
      velocity_setpoint_.z() = direction * 0.7f;
      publishTrajectorySetpoints(goal_, velocity_setpoint_, yaw_setpoint_, yaw_speed_setpoint_);
      ROS_INFO("\033[1;35m [WGN] altitudeChange %f %f %f - %f %f %f \033[0m", goal_.x(), goal_.y(), goal_.z(), velocity_setpoint_.x(), velocity_setpoint_.y(), velocity_setpoint_.z());

      if (explorarion_is_active_) { landing_radius_ = 0.5f; }
      is_within_landing_radius_ = (goal_.topRows<2>() - position_.topRows<2>()).norm() < landing_radius_;
      in_land_vertical_range_ = fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())) < loiter_height_ &&
        fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())) > (loiter_height_ - 0.5f);
      ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ", (goal_.topRows<2>() - position_.topRows<2>()).norm(), fabsf(position_.z() - grid_lsd_.mean_(pos_index_.x(), pos_index_.y())));


      if (is_within_landing_radius_ && in_land_vertical_range_ && is_land_waypoint_) {
        start_seq_landing_decision_ = grid_lsd_seq_;
        prev_lsd_state_ = LSDState::altitudeChange;
        lsd_state_ = LSDState::loiter;
        ROS_INFO("\033[1;34m [WGN] Update to Loiter State \033[0m");
      }
    break;
  }

    case LSDState::loiter: {

      if (prev_lsd_state_ != LSDState::loiter) {
        loiter_position_ = position_;
        loiter_yaw_ = yaw_;
      }

      int offset_center = grid_lsd_.land_.rows() / 2;

      for (int i = offset_center - smoothing_land_cell_; i < offset_center + smoothing_land_cell_; i++) {
        for (int j = offset_center - smoothing_land_cell_; j < offset_center + smoothing_land_cell_; j++) {
          int index = (smoothing_land_cell_ * 2) * (i - offset_center + smoothing_land_cell_) + (j - offset_center + smoothing_land_cell_);
          float cell_land_value = static_cast<float>(grid_lsd_.land_(i, j));
          float can_land_hysteresis_prev = can_land_hysteresis_[index];
          can_land_hysteresis_[index] = (beta_ * can_land_hysteresis_prev) + (1.f - beta_) * cell_land_value;
        }
      }

      if (abs(grid_lsd_seq_ - start_seq_landing_decision_) > 20) {
        decision_taken_ = true;
        int land_counter = 0;
        for (int i = 0; i < can_land_hysteresis_.size(); i++) {
          std::cout << can_land_hysteresis_[i] << " ";
          if (can_land_hysteresis_[i] > can_land_thr_) {
            land_counter++;
          }
          can_land_ = (can_land_ && (can_land_hysteresis_[i] > can_land_thr_));
          if (can_land_ == 0 && land_counter == can_land_hysteresis_.size()) {
            can_land_ = 1;
            decision_taken_ = false;
            in_land_vertical_range_ = false;
            ROS_INFO("[WGN] Decision changed from can't land to can land!");
          }
        }
      }

      publishTrajectorySetpoints(loiter_position_, nan_setpoint, loiter_yaw_, NAN);
      ROS_INFO("\033[1;34m [WGN] Loiter %f %f %f - nan nan nan \033[0m\n", loiter_position_.x(), loiter_position_.y(), loiter_position_.z());

      if (decision_taken_ &&  can_land_) {
        ROS_INFO("\033[1;36m [WGN] Update to Land State \033[0m");
        lsd_state_ = LSDState::land;
      }

      if (decision_taken_ &&  !can_land_) {
        if (!explorarion_is_active_) {
          exploration_anchor_ = loiter_position_;
          explorarion_is_active_ = true;
        }
        float offset_exploration_setpoint = factor_exploration_ * 2.f * static_cast<float>(smoothing_land_cell_) * grid_lsd_.cell_size_;
        n_explored_pattern_++;
        if (n_explored_pattern_ == exploration_pattern.size()) {
          n_explored_pattern_ = 0;
          factor_exploration_ += 1.f;
        }
        goal_ = Eigen::Vector3f(exploration_anchor_.x() + offset_exploration_setpoint * exploration_pattern[n_explored_pattern_].x(),
            exploration_anchor_.y() + offset_exploration_setpoint * exploration_pattern[n_explored_pattern_].y(), exploration_anchor_.z());
        velocity_setpoint_ = nan_setpoint;
        lsd_state_ = LSDState::goTo;
        ROS_INFO("\033[1;32m [WGN] Update to goTo State \033[0m");
      }

      break;
    }

    case LSDState::land:
      loiter_position_.z() = NAN;
      vel_sp = nan_setpoint;
      vel_sp.z() = -.7f;
      publishTrajectorySetpoints(loiter_position_, vel_sp, loiter_yaw_, NAN);
      ROS_INFO("\033[1;36m [WGN] Land %f %f %f - nan nan nan \033[0m\n", loiter_position_.x(), loiter_position_.y(), loiter_position_.z());
      lsd_state_ = LSDState::land;
      break;
  }

}

void WaypointGeneratorNode::updateLSDState() {

  if (update_smoothing_size_) {
    can_land_hysteresis_.resize((smoothing_land_cell_ * 2) * (smoothing_land_cell_ * 2));
    std::fill(can_land_hysteresis_.begin(), can_land_hysteresis_.end(), 0.f);
    update_smoothing_size_ = false;
  }

  if (!is_land_waypoint_) {
    decision_taken_ = false;
    can_land_ = true;
    can_land_hysteresis_.reserve((smoothing_land_cell_ * 2) * (smoothing_land_cell_ * 2));
    std::fill(can_land_hysteresis_.begin(), can_land_hysteresis_.end(), 0.f);
    lsd_state_ = LSDState::goTo;
    explorarion_is_active_ = false;
    ROS_INFO("[WGN] Not a land waypoint");
  }

  return;
}

void WaypointGeneratorNode::publishTrajectorySetpoints(const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp, float yaw_sp, float yaw_speed_sp) {
  mavros_msgs::Trajectory setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "local_origin";
  setpoint.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  setpoint.point_1.position.x = pos_sp.x();
  setpoint.point_1.position.y = pos_sp.y();
  setpoint.point_1.position.z = pos_sp.z();
  setpoint.point_1.velocity.x = vel_sp.x();
  setpoint.point_1.velocity.y = vel_sp.y();
  setpoint.point_1.velocity.z = vel_sp.z();
  setpoint.point_1.acceleration_or_force.x = NAN;
  setpoint.point_1.acceleration_or_force.y = NAN;
  setpoint.point_1.acceleration_or_force.z = NAN;
  setpoint.point_1.yaw = yaw_sp;
  setpoint.point_1.yaw_rate = yaw_speed_sp;

  fillUnusedTrajectorySetpoints(setpoint.point_2);
  fillUnusedTrajectorySetpoints(setpoint.point_3);
  fillUnusedTrajectorySetpoints(setpoint.point_4);
  fillUnusedTrajectorySetpoints(setpoint.point_5);

  setpoint.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  setpoint.point_valid = {true, false, false, false, false};
  trajectory_pub_.publish(setpoint);
}

void WaypointGeneratorNode::fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget& point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

void WaypointGeneratorNode::landingAreaVisualization() {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker cell;
  cell.header.frame_id = "local_origin";
  cell.header.stamp = ros::Time::now();
  cell.id = 0;
  cell.type = visualization_msgs::Marker::CUBE;
  cell.action = visualization_msgs::Marker::ADD;
  cell.pose.orientation.x = 0.0;
  cell.pose.orientation.y = 0.0;
  cell.pose.orientation.z = 0.0;
  cell.pose.orientation.w = 1.0;
  cell.scale.x = grid_lsd_.cell_size_;
  cell.scale.y = grid_lsd_.cell_size_;
  cell.scale.z = 1.0;
  cell.color.a = 0.5;
  cell.scale.z = 0.1;
  cell.color.b = 0.0;

  Eigen::Vector2f grid_min, grid_max;
  grid_lsd_.getGridLimits(grid_min, grid_max);
  int offset = grid_lsd_.land_.rows() / 2;
  float hysteresis_max_value = 1.0f;
  float hysteresis_min_value = 0.0f;
  float range_max = 360.f;
  float range_min = 0.f;
  int counter = 0;

  for (size_t i = 0; i < std::ceil(grid_lsd_.grid_size_ / grid_lsd_.cell_size_); i++) {
    for (size_t j = 0; j < std::ceil(grid_lsd_.grid_size_ / grid_lsd_.cell_size_); j++) {
      if (i>=(offset - smoothing_land_cell_) && i < (offset + smoothing_land_cell_) && j>= (offset - smoothing_land_cell_) && j < (offset + smoothing_land_cell_)) {
        cell.pose.position.x = (i * grid_lsd_.cell_size_) + grid_min.x() + (grid_lsd_.cell_size_ / 2.f);
        cell.pose.position.y = (j * grid_lsd_.cell_size_) + grid_min.y() + (grid_lsd_.cell_size_ / 2.f);
        cell.pose.position.z = 1.0;
        if (can_land_hysteresis_[counter] > can_land_thr_) {
          cell.color.r = 0.0;
          cell.color.g = 1.0;
        } else {
          cell.color.r = 1.0;
          cell.color.g = 0.0;
        }
        cell.color.a = 0.5;
        counter++;

        marker_array.markers.push_back(cell);
        cell.id += 1;
      }

    }
  }
  land_hysteresis_pub_.publish(marker_array);
}

void WaypointGeneratorNode::goalVisualization() {
  visualization_msgs::Marker m;
  static int id = 0;
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 0.5;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;


  m.lifetime = ros::Duration();
  m.id = id;
  id++;
  m.pose.position.x = goal_.x();
  m.pose.position.y = goal_.y();
  m.pose.position.z = goal_.z();


  marker_goal_pub_.publish(m);
}

}

int main(int argc, char **argv) {
  using namespace avoidance;
  ros::init(argc, argv, "waypoint_generator_node");
  ros::NodeHandle nh("~");

  WaypointGeneratorNode NodeWG(nh);
  NodeWG.startNode();
  while (ros::ok()) {
    ros::spin();

  }


  return 0;
}

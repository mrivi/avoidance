#include "safe_landing_planner/safe_landing_planner_visualization_node.hpp"
#include <avoidance/common.h>


#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace avoidance {

SafeLandingPlannerVisualizationNode::SafeLandingPlannerVisualizationNode(const ros::NodeHandle& nh) : nh_(nh) {

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>(
      "/mavros/local_position/pose", 1,
      &SafeLandingPlannerVisualizationNode::positionCallback, this);
  grid_sub_ = nh_.subscribe("/grid_slp", 1, &SafeLandingPlannerVisualizationNode::gridCallback, this);
  mean_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/grid_mean", 1);
  std_dev_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/grid_std_dev", 1);
  counter_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/grid_counter", 1);
}

void SafeLandingPlannerVisualizationNode::startNode() {
  ros::TimerOptions timer_options(
      ros::Duration(spin_dt_),
      boost::bind(&SafeLandingPlannerVisualizationNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);
  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void SafeLandingPlannerVisualizationNode::cmdLoopCallback(const ros::TimerEvent &event) {
  while (!grid_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  publishMean(grid_slp_);
  publishStandardDeviation(grid_slp_);
  publishCounter(grid_slp_);
  grid_received_ = false;

  return;
}

void SafeLandingPlannerVisualizationNode::gridCallback(
    const safe_landing_planner::SLPGridMsg &msg) {
  // grid_slp_seq_ = msg.header.seq;
  if (grid_slp_.getGridSize() != msg.grid_size ||
      grid_slp_.getCellSize() != msg.cell_size) {
    grid_slp_.resize(msg.grid_size, msg.cell_size);
  }

  for (int i = 0; i < msg.mean.layout.dim[0].size; i++) {
    for (int j = 0; j < msg.mean.layout.dim[1].size; j++) {
      grid_slp_.mean_(i, j) =
          msg.mean.data[msg.mean.layout.dim[1].size * i + j];
      grid_slp_.land_(i, j) =
          msg.land.data[msg.mean.layout.dim[1].size * i + j];
      grid_slp_.counter_(i, j) =
          msg.counter.data[msg.counter.layout.dim[1].size * i + j];
      grid_slp_.variance_(i, j) =
          msg.std_dev.data[msg.std_dev.layout.dim[1].size * i + j];
    }
  }
  //
  // pos_index_.x() = static_cast<int>(msg.curr_pos_index.x);
  // pos_index_.y() = static_cast<int>(msg.curr_pos_index.y);


  grid_slp_.setFilterLimits(position_);
  grid_received_ = true;
}

void SafeLandingPlannerVisualizationNode::positionCallback(
    const geometry_msgs::PoseStamped &msg) {
  position_ = avoidance::toEigen(msg.pose.position);

}

void SafeLandingPlannerVisualizationNode::publishMean(const Grid& grid) {
  visualization_msgs::MarkerArray marker_array;

  float cell_size = grid.getCellSize();
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
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;
  cell.color.r = 0.0;
  cell.color.g = 0.0;
  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);

  float variance_max_value = 3.0f;
  float variance_min_value = 0.0f;
  float range_max = 360.f;
  float range_min = 0.f;

  Eigen::MatrixXf mean = grid.getMean();
  int n_lines_padding_ = 5;
  int size = grid.getRowColSize();
  float mean_diff_thr_ = 0.2f;

  Eigen::MatrixXf mean_padded(size + 2 * n_lines_padding_,
                              size + 2 * n_lines_padding_);
  mean_padded.fill(0.f);
  Eigen::MatrixXi mean_accumulator(size + 2 * n_lines_padding_,
                                   size + 2 * n_lines_padding_);
  mean_accumulator.fill(0);
  mean_padded.block(n_lines_padding_, n_lines_padding_, grid.mean_.rows(),
                    grid.mean_.cols()) = grid.mean_;

  for (int i = n_lines_padding_; i < mean.rows() - n_lines_padding_;
       i++) {
    for (int j = n_lines_padding_; j < mean.cols() - n_lines_padding_;
         j++) {
      for (int k = -n_lines_padding_; k <= n_lines_padding_; k++) {
        for (int t = -n_lines_padding_; t <= n_lines_padding_; t++) {
          // land_accumulator(i, j) += land_padded(i + k, j + t);
          float mean_diff =
              std::abs(mean_padded(i, j) - mean_padded(i + k, j + t));
          if (mean_diff > mean_diff_thr_) {
            mean_accumulator(i, j) += 1;
          }
        }
      }
    }
  }

  int max_n_mean_diff_cells_ = 70;
  mean_accumulator = (mean_accumulator.array() <= max_n_mean_diff_cells_)
                         .select(1, mean_accumulator);
  mean_accumulator = (mean_accumulator.array() > max_n_mean_diff_cells_)
                         .select(0, mean_accumulator);
  Eigen::MatrixXi mean_across(mean.rows(), mean.cols());
  mean_across.block(0, 0, mean.rows(), mean.cols()) =
     mean_accumulator.block(n_lines_padding_, n_lines_padding_,
                            mean.rows(), mean.cols());
                            int slc = 6;
int offset = grid.land_.rows() / 2;
  for (size_t i = 0; i < grid.getRowColSize(); i++) {
    for (size_t j = 0; j < grid.getRowColSize(); j++) {
      cell.pose.position.x = (i * cell_size) + grid_min.x() + (cell_size / 2.f);
      cell.pose.position.y = (j * cell_size) + grid_min.y() + (cell_size / 2.f);
      cell.pose.position.z = position_.z();

      if (mean_across(i, j) == 1) {
        cell.color.r= 0.0;
         cell.color.g=1.0;
          cell.color.b=0.0;
      } else {
        cell.color.r= 1.0;
         cell.color.g=0.0;
          cell.color.b=0.0;
      }
      cell.scale.z = 0.1;
      if (i >= (offset - slc) && i < (offset + slc) && j >= (offset - slc) &&
          j < (offset + slc)) {
            cell.scale.z = 0.5;
          }

      // float h = ((range_max - range_min) * (mean(i, j) - variance_min_value) /
      //            (variance_max_value - variance_min_value)) +
      //           range_min;
      // float red, green, blue;
      // float max_aa = 1.f;
      // std::tie(cell.color.r, cell.color.g, cell.color.b) =
      //     HSVtoRGB(std::make_tuple(h, 1.f, 1.f));

      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  mean_pub_.publish(marker_array);
}

void SafeLandingPlannerVisualizationNode::publishCounter(const Grid& grid) {
  visualization_msgs::MarkerArray marker_array;

  float cell_size = grid.getCellSize();
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
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;
  cell.color.r = 0.0;
  cell.color.g = 0.0;
  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);

  float counter_max_value = 350.0f;
  float counter_min_value = 100.0f;
  float range_max = 360.f;
  float range_min = 0.f;

  Eigen::MatrixXi counter = grid.getCounter();

  for (size_t i = 0; i < grid.getRowColSize(); i++) {
    for (size_t j = 0; j < grid.getRowColSize(); j++) {
      cell.pose.position.x = (i * cell_size) + grid_min.x() + (cell_size / 2.f);
      cell.pose.position.y = (j * cell_size) + grid_min.y() + (cell_size / 2.f);
      cell.pose.position.z = position_.z();

      float h = ((range_max - range_min) * ((float)counter(i, j) - counter_min_value) /
                 (counter_max_value - counter_min_value)) +
                range_min;
      float red, green, blue;
      float max_aa = 1.f;
      std::tie(cell.color.r, cell.color.g, cell.color.b) =
          HSVtoRGB(std::make_tuple(h, 1.f, 1.f));

      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  counter_pub_.publish(marker_array);
}

std::tuple<float, float, float> SafeLandingPlannerVisualizationNode::HSVtoRGB(
    std::tuple<float, float, float> hsv) {
  std::tuple<float, float, float> rgb;
  float fC = std::get<2>(hsv) * std::get<1>(hsv);  // fV * fS;  // Chroma
  float fHPrime = fmod(std::get<0>(hsv) / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = std::get<2>(hsv) - fC;

  if (0 <= fHPrime && fHPrime < 1) {
    std::get<0>(rgb) = fC;
    std::get<1>(rgb) = fX;
    std::get<2>(rgb) = 0;
  } else if (1 <= fHPrime && fHPrime < 2) {
    std::get<0>(rgb) = fX;
    std::get<1>(rgb) = fC;
    std::get<2>(rgb) = 0;
  } else if (2 <= fHPrime && fHPrime < 3) {
    std::get<0>(rgb) = 0;
    std::get<1>(rgb) = fC;
    std::get<2>(rgb) = fX;
  } else if (3 <= fHPrime && fHPrime < 4) {
    std::get<0>(rgb) = 0;
    std::get<1>(rgb) = fX;
    std::get<2>(rgb) = fC;
  } else if (4 <= fHPrime && fHPrime < 5) {
    std::get<0>(rgb) = fX;
    std::get<1>(rgb) = 0;
    std::get<2>(rgb) = fC;
  } else if (5 <= fHPrime && fHPrime < 6) {
    std::get<0>(rgb) = fC;
    std::get<1>(rgb) = 0;
    std::get<2>(rgb) = fX;
  } else {
    std::get<0>(rgb) = 0;
    std::get<1>(rgb) = 0;
    std::get<2>(rgb) = 0;
  }

  std::get<0>(rgb) += fM;
  std::get<1>(rgb) += fM;
  std::get<2>(rgb) += fM;

  return rgb;
}

void SafeLandingPlannerVisualizationNode::publishStandardDeviation(
    const Grid& grid) {
  visualization_msgs::MarkerArray marker_array;

  float cell_size = grid.getCellSize();
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
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;

  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);

  Eigen::MatrixXf variance = grid.getVariance();

  float variance_max_value = 0.2f;
  float variance_min_value = 0.0f;
  float range_max = 360.f;
  float range_min = 0.f;

  for (size_t i = 0; i < grid.getRowColSize(); i++) {
    for (size_t j = 0; j < grid.getRowColSize(); j++) {
      cell.pose.position.x = (i * cell_size) + grid_min.x() + (cell_size / 2.f);
      cell.pose.position.y = (j * cell_size) + grid_min.y() + (cell_size / 2.f);
      cell.pose.position.z = position_.z();

      float h = ((range_max - range_min) * ((variance(i, j)) - variance_min_value) / (variance_max_value - variance_min_value)) + range_min;

      std::tie(cell.color.r, cell.color.g, cell.color.b) =
          HSVtoRGB(std::make_tuple(h, 1.f, 1.f));

      // std::cout << "std " << sqrtf(variance(i, j)) << " rgb " << cell.color.r << " " << cell.color.g << " " << cell.color.b << std::endl;

      if ((variance(i, j)) > 0.2f) {
        cell.color.r = 0.0;
        cell.color.g = 0.0;
        cell.color.b = 0.0;
      }

      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  std_dev_pub_.publish(marker_array);
}


}


int main(int argc, char **argv) {
  using namespace avoidance;
  ros::init(argc, argv, "safe_landing_planner_visualization_node");
  ros::NodeHandle nh("~");

  SafeLandingPlannerVisualizationNode NodeWG(nh);
  NodeWG.startNode();
  while (ros::ok()) {
    ros::Duration(1.0).sleep();
  }

  return 0;
}

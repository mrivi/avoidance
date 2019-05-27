#include "landing_site_detection/landing_site_detection_node.hpp"
#include "landing_site_detection/landing_site_detection.hpp"

#include <landing_site_detection/LSDGridMsg.h>

namespace landing_site_detection {

const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);


LandingSiteDetectionNode::LandingSiteDetectionNode(const ros::NodeHandle &nh) : nh_(nh), spin_dt_(0.1) {
  landing_site_detection_.reset(new LandingSiteDetection());

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_));
#endif

  visualizer_.initializePublishers(nh_);

  std::string camera_topic;
  nh_.getParam("pointcloud_topics", camera_topic);

  dynamic_reconfigure::Server<landing_site_detection::LandingSiteDetectionNodeConfig>::CallbackType f;
  f = boost::bind(&LandingSiteDetectionNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>("/mavros/local_position/pose", 1, &LandingSiteDetectionNode::positionCallback, this);
  pointcloud_sub_ = nh_.subscribe<const sensor_msgs::PointCloud2 &>(camera_topic, 1, &LandingSiteDetectionNode::pointCloudCallback, this);

  mavros_system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);
  grid_pub_ = nh_.advertise<landing_site_detection::LSDGridMsg>("/grid_lsd", 1);

  start_time_ = ros::Time::now();

}

void LandingSiteDetectionNode::dynamicReconfigureCallback(landing_site_detection::LandingSiteDetectionNodeConfig& config, uint32_t level) {
  landing_site_detection_->dynamicReconfigureSetParams(config, level);
}

void LandingSiteDetectionNode::positionCallback(const geometry_msgs::PoseStamped &msg) {
  previous_pose_ = current_pose_;
  current_pose_ = msg;
  position_received_ = true;
}

void LandingSiteDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2 &msg) {
  if (tf_listener_.canTransform("/local_origin", msg.header.frame_id, ros::Time(0))) {
    try {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      // transform message to pcl type
      pcl::fromROSMsg(msg, pcl_cloud);

      // remove nan padding
      std::vector<int> dummy_index;
      dummy_index.reserve(pcl_cloud.points.size());
      pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, dummy_index);

      // transform cloud to /local_origin frame
      pcl_ros::transformPointCloud("/local_origin", pcl_cloud, pcl_cloud, tf_listener_);

      landing_site_detection_->cloud_ = std::move(pcl_cloud);
      cloud_transformed_ = true;
      // data_ready_cv_.notify_one();

    } catch (tf::TransformException &ex) {
      ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
    }
  }
}

void LandingSiteDetectionNode::startNode() {
  cmdloop_timer_ = nh_.createTimer(ros::Duration(spin_dt_), boost::bind(&LandingSiteDetectionNode::cmdLoopCallback, this, _1));
}

void LandingSiteDetectionNode::cmdLoopCallback(const ros::TimerEvent& event) {
  status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_ACTIVE);

  ros::Time start_query_position = ros::Time::now();
  while (!cloud_transformed_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    ros::Duration since_query = ros::Time::now() - start_query_position;
    if (since_query > ros::Duration(landing_site_detection_->timeout_termination_)) {
      status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
      publishSystemStatus();
    }
  }

  // Check if all information was received
  ros::Time now = ros::Time::now();
  ros::Duration since_last_algo = now - last_algo_time_;
  ros::Duration since_start = now - start_time_;
  checkFailsafe(since_last_algo, since_start);

  landing_site_detection_->setPose(avoidance::toEigen(current_pose_.pose.position),
                          avoidance::toEigen(current_pose_.pose.orientation));
  landing_site_detection_->runLandingSiteDetection();
  visualizer_.visualizeLandingSiteDetection(*(landing_site_detection_.get()), current_pose_.pose.position, previous_pose_.pose.position);
  publishSerialGrid();
  last_algo_time_ = ros::Time::now();
  cloud_transformed_ = false;

  if (now - t_status_sent_ > ros::Duration(0.2)) publishSystemStatus();

  return;

}

void LandingSiteDetectionNode::checkFailsafe(ros::Duration since_last_algo, ros::Duration since_start) {
  ros::Duration timeout_termination =
    ros::Duration(landing_site_detection_->timeout_termination_);
  ros::Duration timeout_critical =
    ros::Duration(landing_site_detection_->timeout_critical_);

  if (since_last_algo > timeout_termination && since_start > timeout_termination) {
    status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
  } else if (since_last_algo > timeout_critical && since_start > timeout_critical) {
    status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_CRITICAL);
  }
}

void LandingSiteDetectionNode::publishSystemStatus() {
  status_msg_.header.stamp = ros::Time::now();
  status_msg_.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE we need to add a new component
  mavros_system_status_pub_.publish(status_msg_);
  t_status_sent_ = ros::Time::now();
}

void LandingSiteDetectionNode::publishSerialGrid() {
  static int grid_seq = 0;
  Grid prev_grid = landing_site_detection_->getPreviousGrid();
  LSDGridMsg grid;
  grid.header.frame_id = "local_origin";
  grid.header.seq = grid_seq;
  grid.grid_size = prev_grid.grid_size_;
  grid.cell_size = prev_grid.cell_size_;

  grid.mean.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.mean.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.mean.layout.dim[0].label  = "height";
  grid.mean.layout.dim[0].size   = prev_grid.mean_.cols();
  grid.mean.layout.dim[0].stride = prev_grid.mean_.rows() * prev_grid.mean_.cols();

  grid.mean.layout.dim[1].label  = "width";
  grid.mean.layout.dim[1].size   = prev_grid.mean_.rows();
  grid.mean.layout.dim[1].stride = prev_grid.mean_.rows();
  grid.mean.layout.data_offset = 0;

  grid.land.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.land.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.land.layout.dim[0].label  = "height";
  grid.land.layout.dim[0].size   = prev_grid.land_.cols();
  grid.land.layout.dim[0].stride = prev_grid.land_.rows() * prev_grid.land_.cols();

  grid.land.layout.dim[1].label  = "width";
  grid.land.layout.dim[1].size   = prev_grid.land_.rows();
  grid.land.layout.dim[1].stride = prev_grid.land_.rows();
  grid.land.layout.data_offset = 0;

  Eigen::MatrixXf variance = prev_grid.getVariance();
  grid.std_dev.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.std_dev.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.std_dev.layout.dim[0].label  = "height";
  grid.std_dev.layout.dim[0].size   = variance.cols();
  grid.std_dev.layout.dim[0].stride = variance.rows() * variance.cols();

  grid.std_dev.layout.dim[1].label  = "width";
  grid.std_dev.layout.dim[1].size   = variance.rows();
  grid.std_dev.layout.dim[1].stride = variance.rows();
  grid.std_dev.layout.data_offset = 0;

  Eigen::MatrixXi counter = prev_grid.getCounter();
  grid.counter.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.counter.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.counter.layout.dim[0].label  = "height";
  grid.counter.layout.dim[0].size   = counter.cols();
  grid.counter.layout.dim[0].stride = counter.rows() * counter.cols();

  grid.counter.layout.dim[1].label  = "width";
  grid.counter.layout.dim[1].size   = counter.rows();
  grid.counter.layout.dim[1].stride = counter.rows();
  grid.counter.layout.data_offset = 0;

  for (size_t i = 0; i < std::ceil(prev_grid.grid_size_ / prev_grid.cell_size_); i++) {
    for (size_t j = 0; j < std::ceil(prev_grid.grid_size_ / prev_grid.cell_size_); j++) {

      grid.mean.data.push_back(prev_grid.mean_(i,j));
      grid.land.data.push_back(prev_grid.land_(i,j));
      grid.std_dev.data.push_back(sqrtf(variance(i,j)));
      grid.counter.data.push_back(counter(i,j));
    }
  }
  Eigen::Vector2i pos_index = landing_site_detection_->getPositionIndex();
  grid.curr_pos_index.x = static_cast<float>(pos_index.x());
  grid.curr_pos_index.y = static_cast<float>(pos_index.y());

  grid_pub_.publish(grid);
  grid_seq++;
}

}

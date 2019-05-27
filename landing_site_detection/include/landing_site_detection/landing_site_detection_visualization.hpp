#pragma once

#include "landing_site_detection.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

namespace avoidance {

class LandingSiteDetectionVisualization {
public:
  LandingSiteDetectionVisualization() = default;
  ~LandingSiteDetectionVisualization() = default;

  /**
  * @brief      initializes all publishers used for local planner visualization
  **/
  void initializePublishers(ros::NodeHandle& nh);

  /**
  * @brief injects into the LandingSiteDetectionVisualization class, all the data to be visualized
  * @param[in] planner, LandingSiteDetection class
  * @param[in] pos, current vehicle position
  * @param[in] last_pos, previous vehicle position
  **/
  void visualizeLandingSiteDetection(const LandingSiteDetection &planner, const geometry_msgs::Point &pos, const geometry_msgs::Point &last_pos);

private:
  ros::Publisher local_pointcloud_pub_;
  ros::Publisher grid_pub_;
  ros::Publisher path_actual_pub_;
  ros::Publisher mean_pub_;
  ros::Publisher std_dev_pub_;

  int path_length_ = 0;

  /**
  * @brief       Visualization of the actual path of the drone and the path of
  *the waypoint
  * @params[in]  pos, location of the drone at the last timestep
  * @params[in]  last_pos, location of the drone at the previous timestep
  **/
  void publishPaths(const geometry_msgs::Point& pos, const geometry_msgs::Point &last_pos);

  /**
  * @brief       Visualization of the boolean land grid
  * @params[in]  grid, grid data structure
  * @params[in]  pos, vehicle position
  * @param[in]   smoothing_size, kernel size on cell land hysteresis
  **/
  void publishGrid(const Grid& grid, const geometry_msgs::Point& pos, float smoothing_size) const;

  /**
  * @brief      Visualization of the mean values grid
  * @params[in] grid, grid data structure
  **/
  void publishMean(const Grid& grid) ;

  /**
  * @brief      Visualization of the standard deviation values grid
  * @params[in]  grid, grid data structure
  **/
  void publishStandardDeviation(const Grid& grid);

  /**
  * @brief converts from HSV to RGB color space
  * @param[out] fR Red component, range: [0, 1]
  * @param[out] fG Green component, range: [0, 1]
  * @param[out] fB Blue component, range: [0, 1]
  * @param[in] fH Hue component, range: [0, 360]
  * @param[in] fS Hue component, range: [0, 1]
  * @param[in] fV Hue component, range: [0, 1]
  */
  void HSVtoRGB(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV);

};

}

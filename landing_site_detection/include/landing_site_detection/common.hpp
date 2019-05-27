#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_ros/point_cloud.h>


#include <Eigen/Core>
#include <Eigen/Dense>

namespace landing_site_detection {

#define M_PI_F 3.14159265358979323846f
const float DEG_TO_RAD = M_PI_F / 180.f;
const float RAD_TO_DEG = 180.0f / M_PI_F;

Eigen::Vector3f toEigen(const geometry_msgs::Point& p);
Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq);
Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3);
geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& eqf);
pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity);
/**
* @brief     Compute the yaw angle from a quaternion
* @returns   yaw angle in degrees
**/
float getYawFromQuaternion(const Eigen::Quaternionf q);
/**
* @brief     Compute the yaw angle between current position and point
* @returns   angle between two points in rad
**/
float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v);

}

#include "landing_site_detection/common.hpp"

namespace landing_site_detection {

Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq) {
  Eigen::Quaternionf eqf;
  eqf.x() = gmq.x;
  eqf.y() = gmq.y;
  eqf.z() = gmq.z;
  eqf.w() = gmq.w;
  return eqf;
}

Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3) {
  Eigen::Vector3f ev3(v3.x, v3.y, v3.z);
  return ev3;
}

geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& eqf) {
  geometry_msgs::Quaternion q;
  q.x = eqf.x();
  q.y = eqf.y();
  q.z = eqf.z();
  q.w = eqf.w();
  return q;
}

pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity) {
  pcl::PointXYZI p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  p.intensity = intensity;
  return p;
}

float getYawFromQuaternion(const Eigen::Quaternionf q) {
  float siny_cosp = 2.f * (q.w() * q.z() + q.x() * q.y());
  float cosy_cosp = 1.f - 2.f * (q.y() * q.y() + q.z() * q.z());
  float yaw = atan2(siny_cosp, cosy_cosp);

  return yaw * RAD_TO_DEG;
}

float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v) {
  float dx = v.x() - u.x();
  float dy = v.y() - u.y();

  return atan2(dy, dx);
}

}

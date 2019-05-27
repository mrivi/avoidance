#include "landing_site_detection/safe_landing_planner_node.hpp"

int main(int argc, char **argv) {
  using namespace avoidance;
  ros::init(argc, argv, "landing_site_detection_node");
  ros::NodeHandle nh("~");
  SafeLandingPlannerNode NodeLSD(nh);
  NodeLSD.startNode();

  while(ros::ok()) {
    ros::spin();
  }

  return 0;
}

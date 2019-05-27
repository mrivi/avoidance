#include "landing_site_detection/landing_site_detection_node.hpp"
#include "landing_site_detection/waypoint_generator_node.hpp"


int main(int argc, char **argv) {
  using namespace landing_site_detection;
  ros::init(argc, argv, "landing_site_detection_node");
  ros::NodeHandle nh("~");
  LandingSiteDetectionNode NodeLSD(nh);
  NodeLSD.startNode();

  while(ros::ok()) {
    ros::spin();
  }

  return 0;
}

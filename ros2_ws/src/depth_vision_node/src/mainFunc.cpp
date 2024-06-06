#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "syncFrameRecv.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto syncFrameRecvNode = std::make_shared<LUCIDStereo::SyncFrameRecv>();
  cv::namedWindow("view");
  cv::resizeWindow("view", 1600, 600);
  cv::startWindowThread();
  rclcpp::spin(syncFrameRecvNode);
  cv::destroyWindow("view");
  rclcpp::shutdown();
}

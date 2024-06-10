#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "syncFrameRecv.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto syncFrameRecvNode = std::make_shared<LUCIDStereo::SyncFrameRecv>();
  cv::namedWindow("left", 0);
  cv::namedWindow("right", 0);
  cv::namedWindow("disparity", 0);
  cv::startWindowThread();
  rclcpp::spin(syncFrameRecvNode);
  cv::destroyWindow("left");
  cv::destroyWindow("right");
  cv::destroyWindow("disparity");
  rclcpp::shutdown();
}

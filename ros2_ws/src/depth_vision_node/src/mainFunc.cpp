#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "syncFrameRecv.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto syncFrameRecvNode = std::make_shared<LUCIDStereo::SyncFrameRecv>(options);
  rclcpp::spin(syncFrameRecvNode);
  rclcpp::shutdown();
}

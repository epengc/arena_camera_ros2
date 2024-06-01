#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "syncFrameRecv.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto syncFrameRecvNode = std::make_shared<LUCIDStereo::SyncFrameRecv>();
  rclcpp::spin(syncFrameRecvNode);
  rclcpp::shutdown();
}

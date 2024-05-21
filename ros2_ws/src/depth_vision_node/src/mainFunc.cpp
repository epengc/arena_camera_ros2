#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "disparityMap.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto disparityNode = std::make_shared<stereogopro::DisparityNode>(options);
  rclcpp::spin(disparityNode);
  rclcpp::shutdown();
}

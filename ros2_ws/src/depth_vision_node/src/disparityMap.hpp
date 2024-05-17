#ifndef DISPARITYMAP_HPP
#define DISPARITYMAP_HPP
#include <cv_bridge/cv_bridge.h>
#include <rmw/events_statuses/events_statuses.h>

#include <algorithm>
#include <cstdint>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <stereo_msgs/msg/detail/disparity_image__struct.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
using namespace std::placeholders;
typedef std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> disparityParameters;

namespace stereogopro {
class DisparityNode : public rclcpp::Node {
 public:
  explicit DisparityNode(const rclcpp::NodeOptions& options);

 private:
   std::string sub_topic01_name_ = "/arena/left_image_raw";
   std::string sub_topic02_name_ = "/arena/right_image_raw";

   enum StereoAlgorithm { SEMI_GLOBAL_BLOCK_MATCHING };
   // Subscriptions
   void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &l_image_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr &r_image_msg);
};

// add disparity calculation parameters
static void add_param_to_map(disparityParameters parameters,
                             const std::string& name,
                             const std::string& description,
                             const double default_value,
                             const double from_value,
                             const double to_value,
                             const double step) {
  rcl_interfaces::msg::FloatingPointRange floating_point_range;
  floating_point_range.from_value = from_value;
  floating_point_range.to_value = to_value;
  floating_point_range.step = step;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.floating_point_range = {floating_point_range};
  parameters[name] = std::make_pair(default_value, descriptor);
}

DisparityNode::DisparityNode(const rclcpp::NodeOptions& options) :
  Node("DisparityNode", options) {
  // TransportHints does not actually declare the parameter
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " DisparityNode Init With Params ");
  RCLCPP_INFO(get_logger(), "********************************");
  // Declare/read parameters
  this->declare_parameter("queue_size", 5);
  this->declare_parameter("approximate_sync", false);
  this->declare_parameter("use_system_default_qos", false);
  this->declare_parameter<std::string>("intricis_path", "../data/intrinsics.yml");
  this->declare_parameter<std::string>("extricis_path", "../data/extrinsics.yml");
  bool approx = this->get_parameter("approximate_sync", rclcpp::PARAMETER_BOOL);
  int queue_size = this->get_parameter("queue_size", rclcpp::PARAMETER_INTEGER);
  // Synchronize callbacks
  if (approx) {
    approximate_sync_->reset(new ApproximateSync(ApproximatePolicy(queue_size, sub_l_image_, sub_r_image_));
    approximate_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2));
  } else {
    exact_sync_->reset(new ExactSync(ExactPolicy(queue_size, sub_l_image_, sub_r_image_));
    exact_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2));
  }

  // Describe int paramters
  disparityParameters disparity_params;
  add_param_to_map(disparity_params, "stereo_algorithm", "Stereo Algorithm: Block Matching (0) or Semi-Global Block Matching (1)", 0, 0, 1, 1);
  add_param_to_map(disparity_params, "prefilter_size", "Normalization window size in pixels (must be odd)", 9, 5, 255, 2);
  add_param_to_map(disparity_params, "prefilter_cap", "Bound on normalized pixel values", 31, 1, 63, 1);
  add_param_to_map(disparity_params, "corrlation_window_size", "SAD correlation window width in pixels (must be odd)", 15, 5, 255, 2);
  add_param_to_map(disparity_params, "min_disparity", "Disparity to begin search at in pixels", 0, -2048, 2048, 1);
  add_param_to_map(disparity_params, "diparity_range", "Number of disparities to search in pixels (must be a multiple of 16)", 64, 32, 4096, 16);
  add_param_to_map(disparity_params, "texture_threshold", "Filter out if SAD window response does not exceed texture threshold", 10, 0, 10000, 1);
  add_param_to_map(disparity_params, "speckle_size", "Reject regions smaller than this size in pixels", 100, 0, 1000, 1);
  add_param_to_map(disparity_params, "speckle_range", "Maximum allowed difference between detected disparities", 4, 0, 31, 1);
  add_param_to_map(disparity_params, "disp12_max_diff", "Maximum allowed difference in the left-right disparity(Semi-Global Block Matching Only)", 0, 0, 128, 1);
  add_param_to_map(disparity_params, "sgbm_mode", "Mode of the SGBM stereo matcher.", 0, 0, 3, 1);
  add_param_to_map(disparity_params, "uniqueness_ratio", "Filter out if best match does not sufficiently exceed the next-best match ", 15.0, 0.0, 100.0, 0.0);
  add_param_to_map(disparity_params, "P1", "The first parameter controlling the disparity smoothness (Semi-Global Block Matching only)", 200.0, 0.0, 4000.0, 0.0);
  add_param_to_map(disparity_params, "P2", "The second parameter controlling the disparity smoothess (Semi-Global Block Matching only)", 400.0, 0.0, 4000.0, 0.0);

  // Declaring parameters triggers the previously registered callback
  this->declare_parameters<disparityParameters>("disparity_params", disparity_params);

  if (!sub_l_image_.getSubscriber()) {
    // For compressed topics to remap appropriately, we need to pass a fully
    // expended and remapped topic name to image_transport
    auto node_base = this->get_node_base_interface();
    std::string left_topic = node_base->resolve_topic_or_service_name("left/image_rect", false);
    std::string right_topic = node_base->resolve_topic_or_service_name("right/image_rect", false);

    // REP-2003 specifies that subscriber should be SensorDataQos
    const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

    // Support image transpot for compression
    image_transport::TransportHints hints(this);

    // Allow overriding QoS settings (history, depth, reliability)
    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    sub_l_image_.subscribe(this, left_topic, hints.getTransport(), sensor_data_qos, sub_opts);
    sub_r_image_.subscribe(this, right_topic, hints.getTransport(), sensor_data_qos, sub_opts);
  }

  // Publisher Options to Allow Reconfigurable qos settings and connect
  // callback
  rclcpp::PublisherOptions pub_opts;
  pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1, pub_opts);
}

void DisparityNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& l_image_msg, const sensor_msgs::msg::Image::ConstSharedPtr& r_image_msg) {
  // If there are no subscriptions for the disparity image, do nothing
  if (pub_disparity_->get_subscription_count() == 0u) {
    return;
  }
  auto disp_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
  disp_msg->header = l_image_msg->header;
  disp_msg->image.header = l_image_msg->header;
  const cv::Mat_<uint16_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO16)->image;
  const cv::Mat_<uint16_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO16)->image;
}

void DisparityNode::rectFrame(const cv::Mat& left_frame, const cv::Mat& right_frame, cv::Mat& rect_left_frame, cv::Mat& rect_right_frame) {}

}  // namespace stereogopro

#endif

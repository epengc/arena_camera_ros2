#ifndef SYNCFRAMERECV_HPP
#define SYNCFRAMERECV_HPP
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
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
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <utility>
#include <vector>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <opencv2/highgui.hpp>
//typedef std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> disparityParameters;

namespace LUCIDStereo {

// add disparity calculation parameters
//static void add_params(disparityParameters parameters,
//                       const std::string& name,
//                       const std::string& description,
//                       const double default_value,
//                       const double from_value,
//                       const double to_value,
//                       const double step) {
//  rcl_interfaces::msg::FloatingPointRange floating_point_range;
//  floating_point_range.from_value = from_value;
//  floating_point_range.to_value = to_value;
//  floating_point_range.step = step;
//  rcl_interfaces::msg::ParameterDescriptor descriptor;
//  descriptor.description = description;
//  descriptor.floating_point_range = {floating_point_range};
//  parameters[name] = std::make_pair(default_value, descriptor);
//}

class SyncFrameRecv : public rclcpp::Node {
 public:
  explicit SyncFrameRecv() : Node("SyncFrameRecv"){
    using std::placeholders::_1;
  // TransportHints does not actually declare the parameter
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " DisparityNode Init With Params ");
  RCLCPP_INFO(get_logger(), "********************************");
  // Declare parameters of prior-settings
  this->declare_parameter<int>("queue_size", 5);
  this->declare_parameter<bool>("approximate_sync", false);
  this->declare_parameter<bool>("use_system_default_qos", false);
  this->declare_parameter<std::string>("intrincis_path", "../data/intrinsics.yml");
  this->declare_parameter<std::string>("extrincis_path", "../data/extrinsics.yml");
  // Declare parameters of disparityMap work
  //disparityParameters disparity_params;
  //add_params(disparity_params, "stereo_algorithm", "Stereo Algorithm: Block Matching (0) or Semi-Global Block Matching (1)", 0, 0, 1, 1);
  //add_params(disparity_params, "prefilter_size", "Normalization window size in pixels (must be odd)", 9, 5, 255, 2);
  //add_params(disparity_params, "prefilter_cap", "Bound on normalized pixel values", 31, 1, 63, 1);
  //add_params(disparity_params, "corrlation_window_size", "SAD correlation window width in pixels (must be odd)", 15, 5, 255, 2);
  //add_params(disparity_params, "min_disparity", "Disparity to begin search at in pixels", 0, -2048, 2048, 1);
  //add_params(disparity_params, "diparity_range", "Number of disparities to search in pixels (must be a multiple of 16)", 64, 32, 4096, 16);
  //add_params(disparity_params, "texture_threshold", "Filter out if SAD window response does not exceed texture threshold", 10, 0, 10000, 1);
  //add_params(disparity_params, "speckle_size", "Reject regions smaller than this size in pixels", 100, 0, 1000, 1);
  //add_params(disparity_params, "speckle_range", "Maximum allowed difference between detected disparities", 4, 0, 31, 1);
  //add_params(disparity_params, "disp12_max_diff", "Maximum allowed difference in the left-right disparity(Semi-Global Block Matching Only)", 0, 0, 128, 1);
  //add_params(disparity_params, "sgbm_mode", "Mode of the SGBM stereo matcher.", 0, 0, 3, 1);
  //add_params(disparity_params, "uniqueness_ratio", "Filter out if best match does not sufficiently exceed the next-best match ", 15.0, 0.0, 100.0, 0.0);
  //add_params(disparity_params, "P1", "The first parameter controlling the disparity smoothness (Semi-Global Block Matching only)", 200.0, 0.0, 4000.0, 0.0);
  //add_params(disparity_params, "P2", "The second parameter controlling the disparity smoothess (Semi-Global Block Matching only)", 400.0, 0.0, 4000.0, 0.0);
  //// Declaring parameters triggers the previously registered callback
  //this->declare_parameter<disparityParameters>("disparity_params", disparity_params);
  // For avoiding incompatible QoSInitialization by using rclcpp::SensorDataQoS()
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/arena_camera_node/images", rclcpp::SensorDataQoS(), std::bind(&SyncFrameRecv::disparity_publisher_callback,
                                                                                                                                     this,
                                                                                                                                     _1));
  }
 private:
  std::string sub_topic_name_ = "/arena_camera_node/images";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_l_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_r_;
  rclcpp::SensorDataQoS pub_qos_;
  // Subscriptions
  void disparity_publisher_callback(const sensor_msgs::msg::Image &sync_frame_msg);
};


void SyncFrameRecv::disparity_publisher_callback(const sensor_msgs::msg::Image &sync_frame_msg) {
  RCLCPP_INFO(get_logger(), "Prepare for subscription");
  // get sync_frame from ros2 topic and convert them to opencv mat; Format of
  // sync_frame is 4896x2048
  const cv::Mat_<uint16_t> sync_frame_cv_mat = cv_bridge::toCvCopy(sync_frame_msg, sensor_msgs::image_encodings::MONO16)->image;
  // get left and right frame from ros2 topic
  cv::Mat rect_left_frame_cvmat, rect_right_frame_cvmat;
  int width = sync_frame_msg.width;
  int height = sync_frame_msg.height;
  cv::Mat left_frame_cvmat = sync_frame_cv_mat(cv::Rect(0, 0, (int)(width / 2), height));
  cv::Mat right_frame_cvmat = sync_frame_cv_mat(cv::Rect((int)(width / 2), 0, (int)(width / 2), height));
  //cv::Mat left_frame_cvmat = sync_frame_cv_mat;
  //cv::Mat right_frame_cvmat = sync_frame_cv_mat;
  //std::vector<int> compression_params;
  //compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  //compression_params.push_back(9);
  //cv::imwrite("./samples_left.png", left_frame_cvmat, compression_params);
  //cv::imwrite("./samples_right.png", right_frame_cvmat, compression_params);
  std_msgs::msg::Header h = sync_frame_msg.header;
  std::stringstream message;
  message<<h.stamp.sec;
  RCLCPP_INFO(this->get_logger(), "Recving a frame with width '%s'", message.str().c_str());
  publisher_l_ = this->create_publisher<sensor_msgs::msg::Image>("/lucid_camera/left_frame", pub_qos_);
  publisher_r_ = this->create_publisher<sensor_msgs::msg::Image>("/lucid_camera/right_frame", pub_qos_);
  sensor_msgs::msg::Image::SharedPtr msg_l = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_frame_cvmat).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr msg_r = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_frame_cvmat).toImageMsg();
  publisher_l_->publish(std::move(*msg_l));
  publisher_r_->publish(std::move(*msg_r));
  cv::namedWindow("FrameDisplay", cv::WINDOW_NORMAL);
  cv::resizeWindow("FrameDisplay", 600, 600);
  cv::imshow("FrameDisplay", left_frame_cvmat);
  //auto disp_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
  //disp_msg->header = l_image_msg->header;
  //disp_msg->image.header = l_image_msg->header;
}

} // namespace LUCIDStereo

#endif

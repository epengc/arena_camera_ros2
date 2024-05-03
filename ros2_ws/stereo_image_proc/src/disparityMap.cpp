#include <algorithm>
#include <map>
#include <memory>
#include <opencv2/opencv2.hpp>
#include <rcl_interfaces/msg/detail/integer_range__struct.hpp>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <ssteam>
#include <stereo_image_proc/stereo_processor.hpp>
#include <stereo_msgs/msg/disparity_map.hpp>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/stereo_camera_model.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/syn_policies/approximate_epsilon_time.h"
#include "message_filters/syn_policies/approximate_time.h"
#include "message_filters/syn_policies/exact_time.h"
#include "message_filters/synchronizer.h"

using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::images,
                                                              sensor_msgs::msg::CameraInfo,
                                                              sensor_msgs::msg::Image,
                                                              sensor_msgs::msg::CameraInfo>;
using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                          sensor_msgs::msg::CameraInfo,
                                                                          sensor_msgs::msg::Image,
                                                                          sensor_msgs::msg::CameraInfo>;
using ApproximateEpsilonPolicy = message_filters::sync_policies::ApproximateEpsilonTime<sensor_msgs::msg::Image,
                                                                                        sensor_msgs::msg::CameraInfo,
                                                                                        sensor_msgs::msg::Image,
                                                                                        sensor_msgs::msg::CameraInfo>;
using ExactSync = message_filters::Synchronizer<ExactPolicy>;
using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
using ApproximateEpsilonSync = message_filters::Synchronizer<ApproximateEpsilonPolicy>;

namespace stereo_image_proc
{


class DisparityNode : public rclcpp::Node
{
 public:
  explicit DisparityNode(const rclcpp::NodeOptions & options);

 private:
  enum StereoAlgorithm { SEMI_GLOBAL_BLOCK_MATCHING };

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_l_info_, sub_r_info;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  std::shared_ptr<ApproximateEpsilonPolicy> approximate_epsilon_sync_;

  // Publications
  std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisprityImage>> pub_disparity_;
  // handle to parameters callback
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle;
  // Processing state (note: only safe for single-thread)
  image_geometry::StereoCameraModel model_;
  // contains scratch buffers for block matching
  stereo_image_proc::StereoProcessor block_matcher_;

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
               const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg);

  rcl_interfaces::msg::SetParametersResult parameterSetCb(const std::vector<rclcpp::Parameter> & parameters);
};


  // Some helper functions for adding a parameter to a collection
  static void add_param_to_map(std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
                               const std::string & name,
                               const std::string & description,
                               const int default_value,
                               const int from_value,
                               const int to_value,
                               const int step)
  {
    rcl_interfaces::msg::IntegerRange integer_range;
    integer_range.from_value = from_value;
    integer_range.to_value = to_value;
    integer_range.step = step;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.integer_range = {integer_range};
    parameter[name] = std::make_pair(default_value, descriptor);
  }


  static void add_param_to_map(std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
                               const std::string& name,
                               const std::string& description,
                               const double default_value,
                               const double from_value,
                               const double to_value,
                               const double step)
  {
    rcl_interfaces::msg::FloatingPointRange floating_point_range;
    floating_point_range.from_value = from_value;
    floating_point_range.to_value = to_value;
    floating_point_range.step = step;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.floating_point_range = {floating_point_range};
    parameter[name] = std::make_pair(default_value, descriptor);
  }


  DisparityNode::DisparityNode(const rclcpp::NodeOptions & options):rclcpp::Node("diparity_node", options)
  {
    using namespace std::placeholders;
    // TransportHints does not actually declare the parameter
    this->declare_parameter<std::string>("image_transport", "raw");
    // Declare/read parameters
    int queue_size = this->declare_parameters("queue_size", 5);
    bool approx = this->declare_parameters("approximate_sync", false);
    double approx_sync_epsilon = this->declare_parameter("approximate_sync_tolerance_seconds", 0.0);
    this->declare_parameter("use_system_default_qos", false);

    // Synchronize callbacks
    if(approx){
      if(0.0==approx_sync_epsilon){
        approximate_sync_.reset(new ApproximateSync(ApproximateEpsilonPolicy(queue_size)),
                                sub_l_image_,
                                sub_l_info_,
                                sub_r_image_,
                                sub_r_info_);
        approximate_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
      }else{
        approximate_epsilon_sync_.reset(new ApporximateEpsilonSync(ApproximateEpsilonPolicy(queue_size, rclcpp::Duration::from_seconds(approx_sync_epsilon)),
                                                                   sub_l_image_,
                                                                   sub_l_info_,
                                                                   sub_r_image_,
                                                                   sub_r_info_));
        approximate_epsilon_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
      }
    }else{
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size),
                                      sub_l_image,
                                      sub_l_info,
                                      sub_r_image,
                                      sub_r_info));
      exact_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
    }

    // Register a callback for when parameters are set
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DisparityNode::parameterSetCb, this, _1));
    // Describe int paramters
    std::map<std::string, std::pair<int, rcl_interface::msg::ParameterDescriptor>> int_params;

    add_param_to_map(int_params,
                     "stereo_algorithm",
                     "Stereo Algorithm: Block Matching (0) or Semi-Global Block Matching (1)",
                     // Default, from, to, step
                     0, 0, 1, 1);
    add_param_to_map(int_params,
                     "prefilter_size",
                     "Normalization window size in pixels (must be odd)",
                     9, 5, 255, 2);
    add_param_to_map(int_params,
                     "prefilter_cap",
                     "Bound on normalized pixel values",
                     31, 1, 63, 1);
    add_param_to_map(int_params,
                     "corrlation_window_size",
                     "SAD correlation window width in pixels (must be odd)",
                     15, 5, 255, 2);
    add_param_to_map(int_params,
                     "min_disparity",
                     "Disparity to begin search at in pixels",
                     0, -2048, 2048, 1);
    add_param_to_map(int_params,
                     "diparity_range",
                     "Number of disparities to search in pixels (must be a multiple of 16)",
                     64, 32, 4096, 16);
    add_param_to_map(int_params,
                     "texture_threshold",
                     "Filter out if SAD window response does not exceed texture threshold",
                     10, 0, 10000, 1);
    add_param_to_map(int_params,
                     "speckle_size",
                     "Reject regions smaller than this size in pixels",
                     100, 0, 1000, 1);
    add_param_to_map(int_params,
                     "speckle_range",
                     "Maximum allowed difference between detected disparities",
                     4, 0, 31, 1);
    add_param_to_map(int_params,
                     "disp12_max_diff",
                     "Maximum allowed difference in the left-right disparity check in pixels (Semi-Global Block Matching Only)",
                     0, 0, 128, 1);
    add_param_to_map(int_params,
                     "sgbm_mode",
                     "Mode of the SGBM stereo matcher.",
                     0, 0, 3, 1);
    // Describe double parameters
    std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> double_params;
    add_param_to_map(double_params,
                     "uniqueness_ratio",
                     "Filter out if best match does not sufficiently exceed the next-best match ",
                     15.0, 0.0, 100.0, 0.0);
    add_param_to_map(double_params,
                     "P1",
                     "The first parameter controlling the disparity smoothness (Semi-Global Block Matching only)",
                     200.0, 0.0, 4000.0, 0.0);
    add_param_to_map(double_params,
                     "P2",
                     "The second parameter controlling the disparity smoothess (Semi-Global Block Matching only)",
                     400.0, 0.0, 4000.0, 0.0);

    // Declaring parameters triggers the previously registered callback
    this->declare_parameters("", int_params);
    this->declare_parameters("", double_params);

    // Publisher Options to Allow Reconfigurable qos settings and connect callback
    rclcpp::PublisherOptions pub_opts;
    pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_opts.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo & s){
      if(s.current_count == 0){
        sub_l_image_.unsubscribe();
        sub_l_info_.unsubscribe();
        sub_r_image_.unsubscribe();
        sub_r_info_.unsubscribe();
      }else if(!sub_l_image_.getSubscriber()){
        // For compressed topics to remap appropriately, we need to pass a fully expended and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string left_topic = node_base->resolve_topic_or_service_name("left/image_rect", false);
        std::string right_topic = node_base->resolve_topic_or_service_name("right/image_rect", false);
        // Allow also remapping camera_info to something different than default
        std::string left_info_topic = node_base->resolve_topic_or_service_name(image_transport::getCameraInfoTopic(left_topic), false);
        std::string right_info_topic = node_base->resolve_topic_or_service_name(image_transport::getCameraInfoTopic(right_topic), false);

        // REP-2003 specifies that subscriber should be SensorDataQos
        const auto sensor_data_qos = rclcpp::SensorDataQos().get_rmw_qos_profile();

        // Support image transpot for compression
        image_transport::TransportHints hints(this);

        // Allow overriding QoS settings (history, depth, reliability)
        auto sub_opts = rclcpp::SubscriptionOptions();
        sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

        sub_l_image_.subscribe(this, left_topic, hints.getTransport(), sensor_data_qos, sub_opts);
        sub_l_info_.subscribe(this, left_info_topic, hints.getTransport(), sensor_data_qos, sub_opts);
        sub_r_image_.subscribe(this, right_topic, hints.getTransport(), sensor_data_qos, sub_opts);
        sub_r_info_.subscribe(this, right_info_topic, hints.getTransport(), sensor_data_qos, sub_opts);
      }
    };
    pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity", 1, pub_opts);
  }


  void DisparityNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPrt & l_image_msg,
                              const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
                              const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
                              const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg)
  {
    // If there are no subscriptions for the disparity image, do nothing
    if(pub_disparity_->get_subscription_count() == 0u){
      return;
    }

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Allocate new disparity image message
    
  }


}  // namespace stereo_image_proc

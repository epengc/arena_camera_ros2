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

using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                              sensor_msgs::msg::CameraInfo,
                                                              sensor_msgs::msg::Image,
                                                              sensor_msgs::msg::CameraInfo>;
using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                          sensor_msgs::msg::CameraInfo,
                                                                          sensor_msgs::msg::Image,
                                                                          sensor_msgs::msg::CameraInfo>;
using ApproximateEpsilonPolicy = message_filters::sync_policies::ApproximateEpsilonPolicy<sensor_msgs::msg::Image,
                                                                                          sensor_msgs::msg::CameraInfo,
                                                                                          sensor_msgs::msg::Image,
                                                                                          sensor_msgs::msg::CameraInfo>;
using ExactSync = message_filters::Synchronizer<ExactPolicy>;
using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
using ApproximateEpsilonPolicy = message_filters::Synchronizer<ApproximateEpsilonPolicy>;

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



}  // namespace stereo_image_proc

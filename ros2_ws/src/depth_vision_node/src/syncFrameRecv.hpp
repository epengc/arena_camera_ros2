#ifndef SYNCFRAMERECV_HPP
#define SYNCFRAMERECV_HPP
//#include <cv_bridge/cv_bridge.h>
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
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
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

class StereoSingleGpu{
public:
  explicit StereoSingleGpu(int deviceId = 0);
  ~StereoSingleGpu();
  void compute(const cv::Mat& left_frame,
               const cv::Mat& right_frame,
               cv::Mat& disparity);
  cv::Ptr<cv::cuda::StereoSGM> d_alg_;
  //int get_minDisparity(){return minDisparity_};
  //int get_numDisparities(){return numDisparities_};
private:
  int deviceId_;
  cv::cuda::GpuMat d_leftFrame_;
  cv::cuda::GpuMat d_rightFrame_;
  cv::cuda::GpuMat d_disparity_;
  int numDisparities_ = 8;
  int blocksize_ = 0;
  int P1_ = 8;
  int P2_ = 8;
  int preFilterCap_ = 0;
  int minDisparity_ = 360;
  int uniquenessRatio_ = 0;
  int speckleRange_ = 0;
  int speckleWindowSize_ = 0;
  int disp12MaxDiff_ = 0;
  int dispType_ = CV_16S;

};


  StereoSingleGpu::StereoSingleGpu(int deviceId):deviceId_(deviceId){
    cv::cuda::setDevice(deviceId_);
    d_alg_ = cv::cuda::createStereoSGM();
  }


  StereoSingleGpu::~StereoSingleGpu(){
    cv::cuda::setDevice(deviceId_);
    d_leftFrame_.release();
    d_rightFrame_.release();
    d_disparity_.release();
    d_alg_.release();
  }


  void StereoSingleGpu::compute(const cv::Mat& left_frame,
                                const cv::Mat& right_frame,
                                cv::Mat& disparity){
    //RCLCPP_INFO(this->get_logger(), " start disparityMap comput ");
    cv::cuda::setDevice(deviceId_);
    d_leftFrame_.upload(left_frame);
    d_rightFrame_.upload(right_frame);
    d_alg_->compute(d_leftFrame_, d_rightFrame_, d_disparity_);
    d_disparity_.download(disparity);
  }


StereoSingleGpu gpuAlg(0);

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
   std::string intrinsic_filename = "./data/intrinsics.yml";
   std::string extrinsic_filename = "./data/extrinsics.yml";
   cv::FileStorage fs_in(intrinsic_filename, cv::FileStorage::READ);
   if(!fs_in.isOpened()){
     RCLCPP_ERROR(this->get_logger(), "Fail to open intrinsic file.");
   }
   fs_in["M1"] >> M1_;
   fs_in["D1"] >> D1_;
   fs_in["M2"] >> M2_;
   fs_in["D2"] >> D2_;
   cv::FileStorage fs_ex(extrinsic_filename, cv::FileStorage::READ);
   if(!fs_ex.isOpened()){
     RCLCPP_ERROR(this->get_logger(), "Fail to open extrinsic file.");
   }
   fs_ex["R"] >> R_;
   fs_ex["T"] >> T_;
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

  void rectifyFrames(cv::Mat& left_frame, cv::Mat& right_frame, cv::Mat& rect_left_frame, cv::Mat& rect_right_frame, cv::Mat&& dual_frames);
  ~SyncFrameRecv(){
  }

 private:
  cv::Mat M1_, M2_, D1_, D2_;
  cv::Rect roi1_, roi2_;
  cv::Mat R_, T_, R1_, P1_, R2_, P2_, Q_;
  cv::Mat map11_, map12_, map21_, map22_;
  int frame_col_;
  int frame_row_;
  int frame_step_;
  std::string sub_topic_name_ = "/arena_camera_node/images";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::SensorDataQoS pub_qos_;
  // Subscriptions
  void disparity_publisher_callback(const sensor_msgs::msg::Image &sync_frame_msg);
};


  void SyncFrameRecv::disparity_publisher_callback(const sensor_msgs::msg::Image &sync_frame_msg) {
    // RCLCPP_INFO(get_logger(), "subscription is done");
    // get sync_frame from ros2 topic and convert them to opencv mat; Format of
    // sync_frame is 4896x2048
    // const cv::Mat_<uint8_t> sync_frame_cv_mat = cv_bridge::toCvCopy(sync_frame_msg, sensor_msgs::image_encodings::MONO8)->image;
    // get left and right frame from ros2 topic
    // cv::Mat rect_left_frame_cvmat, rect_right_frame_cvmat;
    //int width = sync_frame_msg.width;
    //int height = sync_frame_msg.height;
    //std::vector<int> compression_params;
    //compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    //compression_params.push_back(9);
    //cv::imwrite("./samples_left.png", left_frame_cvmat, compression_params);
    //cv::imwrite("./samples_right.png", right_frame_cvmat, compression_params);
    //std_msgs::msg::Header h = sync_frame_msg.header;
    //std::stringstream message;
    //message<<h.stamp.sec;
    std::stringstream ss;
    ss<<" width = "<< sync_frame_msg.width;
    ss<<" height = "<< sync_frame_msg.height; 
    ss<<" step = "<< sync_frame_msg.step; 
    ss<<" encoding = "<<sync_frame_msg.encoding;
    RCLCPP_INFO(this->get_logger(), "Recved frames in details '%s'", ss.str().c_str());
  
    frame_col_ = sync_frame_msg.width;
    frame_row_ = sync_frame_msg.height;
    frame_step_ = sync_frame_msg.step;
    //try {
      int sync_frame_width = sync_frame_msg.width;
      int sync_frame_height = sync_frame_msg.height;
      int step = sync_frame_msg.step;
      cv::Mat outImg;
      cv::Mat dual_frame(sync_frame_height, step, CV_8UC1, cv::Scalar(0));
      std::memcpy(dual_frame.ptr<uchar>(0), &sync_frame_msg.data[0], step*sync_frame_height);
      //cv::resize(dual_frame, outImg, cv::Size(), 0.5, 0.5);
      //cv::imshow("view", dual_frame);
      cv::Mat left_frame_cvmat(sync_frame_height, (int)(step/2), CV_8UC1, cv::Scalar(0));
      cv::Mat right_frame_cvmat(sync_frame_height, (int)(step/2), CV_8UC1, cv::Scalar(0));
      cv::Mat rect_left_frame_cvmat, rect_right_frame_cvmat;
      rectifyFrames(left_frame_cvmat,
                    right_frame_cvmat,
                    rect_left_frame_cvmat,
                    rect_right_frame_cvmat,
                    std::move(dual_frame));
      //cv::resize(left_frame_cvmat, outImg, cv::Size(), 1, 1);
      cv::imshow("left", rect_left_frame_cvmat);
      //cv::resize(right_frame_cvmat, outImg, cv::Size(), 1, 1);
      cv::imshow("right",rect_right_frame_cvmat);
      //cv::imshow("view", );
      cv::Mat disp, disparity;
      cv::Mat left = cv::imread("./data/RectLeft.tiff", cv::IMREAD_GRAYSCALE);
      cv::Mat right = cv::imread("./data/RectRight.tiff", cv::IMREAD_GRAYSCALE);
      std::cout<<"left frame rows = "<<rect_left_frame_cvmat.rows<<std::endl;
      std::cout<<"left frame cols = "<<rect_left_frame_cvmat.cols<<std::endl;
      std::cout<<"right frame rows = "<<rect_right_frame_cvmat.rows<<std::endl;
      std::cout<<"right frame rows = "<<rect_right_frame_cvmat.cols<<std::endl;
      gpuAlg.compute(left, right, disp);
      disp.convertTo(disparity, CV_32F, 1.0);
      disparity = (disparity/16.0f-(float)80)/(float)8;
      cv::imshow("disparity", disparity);
      //} catch (...) {
      //RCLCPP_INFO(this->get_logger(), "Fail to copy sensor_msgs::msg::Image.data to cv::Mat ");
      //}
    //publisher_l_ = this->create_publisher<sensor_msgs::msg::Image>("/lucid_camera/left_frame", pub_qos_);
    //publisher_r_ = this->create_publisher<sensor_msgs::msg::Image>("/lucid_camera/right_frame", pub_qos_);
    //sensor_msgs::msg::Image::SharedPtr msg_l = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_frame_cvmat).toImageMsg();
    //sensor_msgs::msg::Image::SharedPtr msg_r = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_frame_cvmat).toImageMsg();
    //publisher_l_->publish(std::move(*msg_l));
    //publisher_r_->publish(std::move(*msg_r));
    //cv::namedWindow("FrameDisplay", cv::WINDOW_NORMAL);
    //cv::resizeWindow("FrameDisplay", 600, 600);
    //cv::imshow("FrameDisplay", left_frame_cvmat);
    //auto disp_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
    //disp_msg->header = l_image_msg->header;
    //disp_msg->image.header = l_image_msg->header;
  }


  void SyncFrameRecv::rectifyFrames(cv::Mat& left_frame,
                                    cv::Mat& right_frame,
                                    cv::Mat& rect_left_frame,
                                    cv::Mat& rect_right_frame,
                                    cv::Mat&& dual_frame){
    int width = dual_frame.cols;
    int height = dual_frame.rows;
    for(int col=0; col<(int)(width/2); col++){
      for(int row=0; row<height; row++){
        right_frame.at<uint8_t>(row, col) = dual_frame.at<uint8_t>(row, col*2);
        left_frame.at<uint8_t>(row, col) = dual_frame.at<uint8_t>(row, col*2+1);
      }
    }
    cv::stereoRectify(M1_, D1_, M2_, D2_, left_frame.size(), R_, T_, R1_, R2_, P1_, P2_, Q_, cv::CALIB_ZERO_DISPARITY, -1, right_frame.size(), &roi1_, &roi2_);
    cv::initUndistortRectifyMap(M1_, D1_, R1_, P1_, left_frame.size(), CV_16SC2, map11_, map12_);
    cv::initUndistortRectifyMap(M2_, D2_, R2_, P2_, right_frame.size(), CV_16SC2, map21_, map22_);
    cv::remap(left_frame, rect_left_frame, map11_, map12_, cv::INTER_LINEAR);
    cv::remap(right_frame, rect_right_frame, map21_, map22_, cv::INTER_LINEAR);
  }




} // namespace LUCIDStereo

#endif

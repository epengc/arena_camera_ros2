#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "syncFrameRecv.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto syncFrameRecvNode = std::make_shared<LUCIDStereo::SyncFrameRecv>();
  cv::namedWindow("left", 0);
  cv::namedWindow("right", 0);
  cv::namedWindow("disparity", cv::WINDOW_NORMAL);
  //cv::createTrackbar("numDisparities",    "disparity", &LUCIDStereo::numDisparities,    32,  LUCIDStereo::on_trackbar1);
  //cv::createTrackbar("blockSize",         "disparity", &LUCIDStereo::blockSize,         50,  LUCIDStereo::on_trackbar2);
  cv::createTrackbar("P1",                "disparity", &LUCIDStereo::P1,                256, LUCIDStereo::on_trackbar3);
  cv::createTrackbar("P2",                "disparity", &LUCIDStereo::P2,                256, LUCIDStereo::on_trackbar4);
  //cv::createTrackbar("preFilterCap",      "disparity", &LUCIDStereo::preFilterCap,      62,  LUCIDStereo::on_trackbar5);
  cv::createTrackbar("minDisparity",      "disparity", &LUCIDStereo::minDisparity,      512, LUCIDStereo::on_trackbar6);
  //cv::createTrackbar("uniquenessRatio",   "disparity", &LUCIDStereo::uniquenessRatio,   100, LUCIDStereo::on_trackbar7);
  //cv::createTrackbar("speckleRange",      "disparity", &LUCIDStereo::speckleRange,      100, LUCIDStereo::on_trackbar8);
  //cv::createTrackbar("speckleWindowSize", "disparity", &LUCIDStereo::speckleWindowSize, 25,  LUCIDStereo::on_trackbar9);
  //cv::createTrackbar("dips12MaxDiff",     "disparity", &LUCIDStereo::disp12MaxDiff,     25,  LUCIDStereo::on_trackbar10);
  cv::startWindowThread();
  rclcpp::spin(syncFrameRecvNode);
  cv::destroyWindow("left");
  cv::destroyWindow("right");
  cv::destroyWindow("disparity");
  rclcpp::shutdown();
}

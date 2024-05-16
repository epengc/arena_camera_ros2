#ifndef DISPARITY_CALCULATION_HPP
#define DISPARITY_CALCULATION_HPP
#include <math.h>
#include <opencv2/core/hal/interface.h>
#include <stdio.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "opencv2/imgcodecs.hpp"

bool rectification(const cv::Mat &left_frame, const cv::Mat &right_frame, cv::Mat &left_frame_rect, cv::Mat &right_frame_rect) {
  std::string intrinsic_filename = "../data/intrinsics.yml";
  std::string extrinsic_filename = "../data/extrinsics.yml";
  // reading intrinsic parameters
  cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error(intrinsic_filename.c_str());
  }
  cv::Mat M1, D1, M2, D2;
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  // reading extrinsic parameters
  fs.open(extrinsic_filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error(extrinsic_filename.c_str());
  }
  cv::Rect roi1, roi2;
  cv::Mat R, T, R1, P1, R2, P2, Q;
  fs["R"] >> R;
  fs["T"] >> T;

  cv::stereoRectify(M1, D1, M2, D2, left_frame.size(), R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, right_frame.size(), &roi1, &roi2);

  cv::Mat map11, map12, map21, map22;
  cv::initUndistortRectifyMap(M1, D1, R1, P1, left_frame.size(), CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, right_frame.size(), CV_16SC2, map21, map22);
  cv::remap(left_frame, left_frame_rect, map11, map12, cv::INTER_LINEAR);
  cv::remap(right_frame, right_frame_rect, map21, map22, cv::INTER_LINEAR);
  return 0;
}

class StereoSingleGpu {
 public:
  explicit StereoSingleGpu(int deviceId = 0);
  ~StereoSingleGpu();
  void compute(const cv::Mat &leftFrame, const cv::Mat &rightFrame, cv::Mat &disparity);
  cv::Ptr<cv::cuda::StereoSGM> d_alg;

 private:
  int deviceId_;
  cv::cuda::GpuMat d_leftFrame;
  cv::cuda::GpuMat d_rightFrame;
  cv::cuda::GpuMat d_diparity;
  int numDisparities_ = 8;
  int blockSize_ = 0;
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

StereoSingleGpu::StereoSingleGpu(int deviceId) : deviceId_(deviceId) {
  cv::cuda::setDevice(deviceId_);
  d_alg = cv::cuda::createStereoSGM();
}

StereoSingleGpu::~StereoSingleGpu() {
  cv::cuda::setDevice(deviceId_);
  d_leftFrame.release();
  d_rightFrame.release();
  d_diparity.release();
  d_alg.release();
}

void StereoSingleGpu::compute(const cv::Mat &leftFrame, const cv::Mat &rightFrame, cv::Mat &disparity) {
  cv::cuda::setDevice(deviceId_);
  d_leftFrame.upload(leftFrame);
  d_rightFrame.upload(rightFrame);
  d_alg->compute(d_leftFrame, d_rightFrame, d_diparity);
  d_diparity.download(disparity);
}
// Creating an object of StereoSGBM algorithm

StereoSingleGpu gpu0Alg(0);
//
// cv::Mat imgL;
// cv::Mat imgR;
// cv::Mat imgL_gray;
// cv::Mat imgR_gray;
//
//// Defining callback functions for the trackbars to update parameter values
//
static void on_trackbar1(int, void *) {
  gpu0Alg.d_alg->setNumDisparities(numDisparities * 16);
  numDisparities = numDisparities * 16;
}

static void on_trackbar2(int, void *) {
  gpu0Alg.d_alg->setBlockSize(blockSize * 2 + 5);
  blockSize = blockSize * 2 + 5;
}

static void on_trackbar3(int, void *) { gpu0Alg.d_alg->setP1(P1); }

static void on_trackbar4(int, void *) { gpu0Alg.d_alg->setP2(P2); }

static void on_trackbar5(int, void *) { gpu0Alg.d_alg->setPreFilterCap(preFilterCap); }

static void on_trackbar6(int, void *) { gpu0Alg.d_alg->setMinDisparity(minDisparity); }

static void on_trackbar7(int, void *) { gpu0Alg.d_alg->setUniquenessRatio(uniquenessRatio); }

static void on_trackbar8(int, void *) { gpu0Alg.d_alg->setSpeckleRange(speckleRange); }

static void on_trackbar9(int, void *) {
  gpu0Alg.d_alg->setSpeckleWindowSize(speckleWindowSize * 2);
  speckleWindowSize = speckleWindowSize * 2;
}

static void on_trackbar10(int, void *) { gpu0Alg.d_alg->setDisp12MaxDiff(disp12MaxDiff); }

int main() {
  // Initialize variables to store the maps for stereo rectification
  // cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
  // cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

  //// Reading the mapping values for stereo image rectification
  // cv::FileStorage cv_file2 = cv::FileStorage("data/stereo_rectify_maps.xml", cv::FileStorage::READ);
  // cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
  // cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
  // cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
  // cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
  // cv_file2.release();

  //// Check for left and right camera IDs
  //// These values can change depending on the system
  // int CamL_id{2}; // Camera ID for left camera
  // int CamR_id{0}; // Camera ID for right camera

  // cv::VideoCapture camL(CamL_id), camR(CamR_id);

  //// Check if left camera is attached
  // if (!camL.isOpened())
  //{
  //   std::cout << "Could not open camera with index : " << CamL_id << std::endl;
  //   return -1;
  // }

  //// Check if right camera is attached
  // if (!camL.isOpened())
  //{
  //   std::cout << "Could not open camera with index : " << CamL_id << std::endl;
  //   return -1;
  // }

  // Creating a named window to be linked to the trackbars
  cv::namedWindow("disparity", cv::WINDOW_NORMAL);
  cv::resizeWindow("disparity", 600, 600);

  // Creating trackbars to dynamically update the StereoBM parameters
  cv::createTrackbar("numDisparities", "disparity", &numDisparities, 32, on_trackbar1);
  cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
  cv::createTrackbar("P1", "disparity", &P1, 150, on_trackbar3);
  cv::createTrackbar("P2", "disparity", &P2, 150, on_trackbar4);
  cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
  cv::createTrackbar("minDisparity", "disparity", &minDisparity, 400, on_trackbar6);
  cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
  cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
  cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
  cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);

  cv::Mat disp, disparity;

  while (true) {
    // Capturing and storing left and right camera images
    // camL >> imgL;
    // camR >> imgR;

    // Converting images to grayscale
    // cv::cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
    // cv::cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);

    // Initialize matrix for rectified stereo images
    cv::Mat left_frame, right_frame, left_frame_rect, right_frame_rect;
    // left_frame = cv::imread("../data/RectLeft.tiff", cv::IMREAD_GRAYSCALE);
    // right_frame = cv::imread("../data/RectRight.tiff", cv::IMREAD_GRAYSCALE);
    left_frame = cv::imread("../data/c01_left.tiff", cv::IMREAD_GRAYSCALE);
    right_frame = cv::imread("../data/c01_right.tiff", cv::IMREAD_GRAYSCALE);

    std::cout << "------sofarsogood-----" << std::endl;
    bool rectification_bool = rectification(left_frame, right_frame, left_frame_rect, right_frame_rect);
    auto start = std::chrono::steady_clock::now();
    gpu0Alg.compute(left_frame_rect, right_frame_rect, disp);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
    // NOTE: Code returns a 16bit signed single channel image,
    // CV_16S containing a disparity map scaled by 16. Hence it
    // is essential to convert it to CV_32F and scale it down 16 times.
    // Converting disparity values to CV_32F from CV_16S
    disp.convertTo(disparity, CV_32F, 1.0);

    // Scaling down the disparity values and normalizing them
    disparity = (disparity / 16.0f - (float)minDisparity) / ((float)numDisparities);

    // Displaying the disparity map
    cv::imshow("disparity", disparity);

    // Close window using esc key
    if (cv::waitKey(1) == 27) break;
  }
  return 0;
}

#endif

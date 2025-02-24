/*
  Copyright (c) 2025 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#include "computer_vision/CVSubscriber.hpp"

namespace computer_vision
{

/**
   TO-DO: Default - the output images are half the size of the input and pointcloud are the same as the input
 */
CVGroup CVSubscriber::processing(
  const cv::Mat color,
  const cv::Mat depth,
  const cv::Mat disparity,
  const cv::Mat left_rect,
  const cv::Mat right_rect,
  const cv::Mat left_raw,
  const cv::Mat right_raw,
  const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
const
{
  // Create output images
  cv::Mat out_color, out_depth, out_disparity, out_left_rect, out_right_rect, out_left_raw,
    out_right_raw;
  // Create output pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;

  // Processing - important: check if the input images are empty before to process them
  if (!color.empty()) {
    cv::resize(color, out_color, cv::Size(), 0.5, 0.5);
    cv::imshow("out_image_color", out_color);
  }
  if (!depth.empty()) {
    cv::resize(depth, out_depth, cv::Size(), 0.5, 0.5);
    cv::imshow("out_image_depth", out_depth);
  }
  if (!disparity.empty()) {
    cv::resize(disparity, out_disparity, cv::Size(), 0.5, 0.5);
    cv::imshow("out_disparity", out_disparity);
  }
  if (!left_rect.empty()) {
    cv::resize(left_rect, out_left_rect, cv::Size(), 0.5, 0.5);
    cv::imshow("out_left_rect", out_left_rect);
  }
  if (!right_rect.empty()) {
    cv::resize(right_rect, out_right_rect, cv::Size(), 0.5, 0.5);
    cv::imshow("out_right_rect", out_right_rect);
  }
  if (!left_raw.empty()) {
    cv::resize(left_raw, out_left_raw, cv::Size(), 0.5, 0.5);
    cv::imshow("out_left_raw", out_left_raw);
  }
  if (!right_raw.empty()) {
    cv::resize(right_raw, out_right_raw, cv::Size(), 0.5, 0.5);
    cv::imshow("out_right_raw", out_right_raw);
  }
  if (!in_pointcloud.empty()) {
    out_pointcloud = in_pointcloud;
  }

  cv::waitKey(3);

  return CVGroup(out_color, out_depth, out_disparity, out_left_rect, out_right_rect,
    out_left_raw, out_right_raw, out_pointcloud);
}

} // namespace computer_vision

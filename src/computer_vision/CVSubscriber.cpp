/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#include "computer_vision/CVSubscriber.hpp"

namespace computer_vision
{

/**
   TO-DO: Default - the output images and pointcloud are the same as the input
 */
CVGroup CVSubscriber::processing(
  const cv::Mat in_image_rgb,
  const cv::Mat in_image_depth,
  const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
const
{
  // Create output images
  cv::Mat out_image_rgb, out_image_depth;
  // Create output pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;

  // Processing
  out_image_rgb = in_image_rgb;
  out_image_depth = in_image_depth;
  out_pointcloud = in_pointcloud;

  // Show images in a different windows
  cv::imshow("out_image_rgb", out_image_rgb);
  cv::imshow("out_image_depth", out_image_depth);
  cv::waitKey(3);

  return CVGroup(out_image_rgb, out_image_depth, out_pointcloud);
}

} // namespace computer_vision

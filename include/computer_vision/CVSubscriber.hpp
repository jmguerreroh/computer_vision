/*
  Copyright (c) 2025 José Miguel Guerrero Hernández

  This file is licensed under the terms of the MIT license.
  See the LICENSE file in the root of this repository
*/

#ifndef INCLUDE_COMPUTER_VISION__CVSUBSCRIBER_HPP_
#define INCLUDE_COMPUTER_VISION__CVSUBSCRIBER_HPP_

#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "pcl/point_types.h"
#include "pcl/point_types_conversion.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace computer_vision
{

class CVGroup
{
public:
  CVGroup(
    cv::Mat rgb,
    cv::Mat depth, cv::Mat disparity,
    cv::Mat left_rect, cv::Mat right_rect,
    cv::Mat left_raw, cv::Mat right_raw,
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud)
  {
    rgb_ = rgb;
    depth_ = depth;
    disparity_ = disparity;
    left_rect_ = left_rect;
    right_rect_ = right_rect;
    left_raw_ = left_raw;
    right_raw_ = right_raw;
    pointcloud_ = pointcloud;
  }
  cv::Mat getImageRGB() {return rgb_;}
  cv::Mat getImageDepth() {return depth_;}
  cv::Mat getImageDisparity() {return disparity_;}
  cv::Mat getImageLeftRect() {return left_rect_;}
  cv::Mat getImageRightRect() {return right_rect_;}
  cv::Mat getImageLeftRaw() {return left_raw_;}
  cv::Mat getImageRightRaw() {return right_raw_;}
  pcl::PointCloud<pcl::PointXYZRGB> getPointCloud() {return pointcloud_;}

private:
  cv::Mat rgb_, depth_, disparity_, left_rect_, right_rect_, left_raw_, right_raw_;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_;
};

class CVSubscriber : public rclcpp::Node
{
public:
  CVSubscriber()
  : Node("cv_subscriber")
  {
    subscription_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 1,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
        RCLCPP_INFO(get_logger(), "Camera info received");
        camera_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
        camera_model_->fromCameraInfo(*msg);
        subscription_info_ = nullptr;
      });

    subscription_rgb_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgb_in", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        last_rgb_ = std::move(msg);
      });

    subscription_depth_ = create_subscription<sensor_msgs::msg::Image>(
      "/depth_in", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        last_depth_ = std::move(msg);
      });

    subscription_disparity_ = create_subscription<stereo_msgs::msg::DisparityImage>(
      "/disparity_in", 1,
      [this](stereo_msgs::msg::DisparityImage::UniquePtr msg) {
        last_disparity_ = std::move(msg);
      });

    subscription_left_rect_ = create_subscription<sensor_msgs::msg::Image>(
      "/left_rect_in", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        last_left_rect_ = std::move(msg);
      });

    subscription_right_rect_ = create_subscription<sensor_msgs::msg::Image>(
      "/right_rect_in", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        last_right_rect_ = std::move(msg);
      });

    subscription_left_raw_ = create_subscription<sensor_msgs::msg::Image>(
      "/left_raw_in", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        last_left_raw_ = std::move(msg);
      });

    subscription_right_raw_ = create_subscription<sensor_msgs::msg::Image>(
      "/right_raw_in", 1,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        last_right_raw_ = std::move(msg);
      });

    subscription_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud_in", 1,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        last_pointcloud_ = std::move(msg);
      });

    publisher_rgb_ = this->create_publisher<sensor_msgs::msg::Image>(
      "rgb",
      rclcpp::SensorDataQoS().reliable());

    publisher_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
      "depth",
      rclcpp::SensorDataQoS().reliable());

    publisher_disparity_ = this->create_publisher<sensor_msgs::msg::Image>(
      "disparity",
      rclcpp::SensorDataQoS().reliable());

    publisher_left_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
      "left_raw",
      rclcpp::SensorDataQoS().reliable());

    publisher_right_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
      "right_raw",
      rclcpp::SensorDataQoS().reliable());

    publisher_left_rect_ = this->create_publisher<sensor_msgs::msg::Image>(
      "left_rect",
      rclcpp::SensorDataQoS().reliable());

    publisher_right_rect_ = this->create_publisher<sensor_msgs::msg::Image>(
      "right_rect",
      rclcpp::SensorDataQoS().reliable());

    publisher_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud",
      rclcpp::SensorDataQoS().reliable());

    // 30Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CVSubscriber::timer_callback, this));
  }

private:
  CVGroup processing(
    const cv::Mat rgb,
    const cv::Mat depth,
    const cv::Mat disparity,
    const cv::Mat left_rect,
    const cv::Mat right_rect,
    const cv::Mat left_raw,
    const cv::Mat right_raw,
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
  const;

  void timer_callback()
  {
     // Check if camera model has been received
    if (camera_model_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model not yet available");
      return;
    }

    // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
    cv_bridge::CvImagePtr image_rgb_ptr, image_depth_ptr, image_disparity_ptr,
      image_left_rect_ptr, image_right_rect_ptr, image_left_raw_ptr, image_right_raw_ptr;

    cv::Mat image_rgb, image_depth, image_disparity, image_left_rect, image_right_rect,
      image_left_raw, image_right_raw;

    try {
      if (last_rgb_ != nullptr) {
        image_rgb_ptr = cv_bridge::toCvCopy(*last_rgb_, sensor_msgs::image_encodings::BGR8);
        image_rgb = image_rgb_ptr->image;
        if (counter_rgb_ == 0) {
          RCLCPP_INFO(get_logger(), "RGB image received");
          counter_rgb_++;
        }
      }

      if (last_depth_ != nullptr) {
        if (last_depth_->encoding != "16UC1") {
          RCLCPP_ERROR(get_logger(), "The depth image type has not 16UC1 encoding: %s",
            (last_depth_->encoding).c_str());
          return;
        }
        image_depth_ptr = cv_bridge::toCvCopy(*last_depth_,
            sensor_msgs::image_encodings::TYPE_16UC1);
        image_depth = image_depth_ptr->image;
        if (counter_depth_ == 0) {
          RCLCPP_INFO(get_logger(), "Depth image received");
          counter_depth_++;
        }
      }

      if (last_disparity_ != nullptr) {
        if (last_disparity_->image.encoding != "32FC1") {
          RCLCPP_ERROR(get_logger(), "The disparity image type has not 16UC1 encoding: %s",
            (last_disparity_->image.encoding).c_str());
          return;
        }
        image_disparity_ptr = cv_bridge::toCvCopy(last_disparity_->image,
            sensor_msgs::image_encodings::TYPE_32FC1);
        image_disparity = image_disparity_ptr->image;
        if (counter_disparity_ == 0) {
          RCLCPP_INFO(get_logger(), "Disparity image received");
          counter_disparity_++;
        }
      }

      if (last_left_rect_ != nullptr) {
        image_left_rect_ptr = cv_bridge::toCvCopy(*last_left_rect_,
            sensor_msgs::image_encodings::BGR8);
        image_left_rect = image_left_rect_ptr->image;
        if (counter_left_rect_ == 0) {
          RCLCPP_INFO(get_logger(), "Left rectified image received");
          counter_left_rect_++;
        }
      }

      if (last_right_rect_ != nullptr) {
        image_right_rect_ptr = cv_bridge::toCvCopy(*last_right_rect_,
            sensor_msgs::image_encodings::BGR8);
        image_right_rect = image_right_rect_ptr->image;
        if (counter_right_rect_ == 0) {
          RCLCPP_INFO(get_logger(), "Right rectified image received");
          counter_right_rect_++;
        }
      }

      if (last_left_raw_ != nullptr) {
        image_left_raw_ptr = cv_bridge::toCvCopy(*last_left_raw_,
            sensor_msgs::image_encodings::BGR8);
        image_left_raw = image_left_raw_ptr->image;
        if (counter_left_raw_ == 0) {
          RCLCPP_INFO(get_logger(), "Left raw image received");
          counter_left_raw_++;
        }
      }

      if (last_right_raw_ != nullptr) {
        image_right_raw_ptr = cv_bridge::toCvCopy(*last_right_raw_,
            sensor_msgs::image_encodings::BGR8);
        image_right_raw = image_right_raw_ptr->image;
        if (counter_right_raw_ == 0) {
          RCLCPP_INFO(get_logger(), "Right raw image received");
          counter_right_raw_++;
        }
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    if (last_pointcloud_ != nullptr) {
      pcl::fromROSMsg(*last_pointcloud_, pointcloud);
      if (counter_pointcloud_ == 0) {
        RCLCPP_INFO(get_logger(), "Pointcloud received");
        counter_pointcloud_++;
      }
    }

    // Image and PointCloud processing
    CVGroup cvgroup = processing(image_rgb, image_depth, image_disparity,
      image_left_rect, image_right_rect, image_left_raw, image_right_raw, pointcloud);

    if (!cvgroup.getImageRGB().empty()) {
      // Convert OpenCV Image to ROS Image
      cv_bridge::CvImage image_rgb_bridge =
        cv_bridge::CvImage(
        last_rgb_->header, sensor_msgs::image_encodings::BGR8,
          cvgroup.getImageRGB());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_rgb;
      // from cv_bridge to sensor_msgs::Image
      image_rgb_bridge.toImageMsg(out_image_rgb);
      // Publish the data
      publisher_rgb_->publish(out_image_rgb);
    }

    if (!cvgroup.getImageDepth().empty()) {
      // Convert to ROS data type
      cv_bridge::CvImage image_depth_bridge =
        cv_bridge::CvImage(
        last_depth_->header, sensor_msgs::image_encodings::TYPE_16UC1,
          cvgroup.getImageDepth());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_depth;
      // from cv_bridge to sensor_msgs::Image
      image_depth_bridge.toImageMsg(out_image_depth);
      // Publish the data
      publisher_depth_->publish(out_image_depth);
    }

    if (!cvgroup.getImageDisparity().empty()) {
      // Convert to ROS data type
      cv_bridge::CvImage image_disparity_bridge =
        cv_bridge::CvImage(
        last_disparity_->image.header, sensor_msgs::image_encodings::TYPE_32FC1,
          cvgroup.getImageDisparity());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_disparity;
      // from cv_bridge to sensor_msgs::Image
      image_disparity_bridge.toImageMsg(out_image_disparity);
      // out_image_disparity.is_bigendian = true;
      // Publish the data
      publisher_disparity_->publish(out_image_disparity);
    }

    if (!cvgroup.getImageLeftRect().empty()) {
      // Convert to ROS data type
      cv_bridge::CvImage image_left_rect_bridge =
        cv_bridge::CvImage(
        last_left_rect_->header, sensor_msgs::image_encodings::BGR8,
          cvgroup.getImageLeftRect());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_left_rect;
      // from cv_bridge to sensor_msgs::Image
      image_left_rect_bridge.toImageMsg(out_image_left_rect);
      // Publish the data
      publisher_left_rect_->publish(out_image_left_rect);
    }

    if (!cvgroup.getImageRightRect().empty()) {
      // Convert to ROS data type
      cv_bridge::CvImage image_right_rect_bridge =
        cv_bridge::CvImage(
        last_right_rect_->header, sensor_msgs::image_encodings::BGR8,
          cvgroup.getImageRightRect());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_right_rect;
      // from cv_bridge to sensor_msgs::Image
      image_right_rect_bridge.toImageMsg(out_image_right_rect);
      // Publish the data
      publisher_right_rect_->publish(out_image_right_rect);
    }

    if (!cvgroup.getImageLeftRaw().empty()) {
      // Convert to ROS data type
      cv_bridge::CvImage image_left_raw_bridge =
        cv_bridge::CvImage(
        last_left_raw_->header, sensor_msgs::image_encodings::BGR8,
          cvgroup.getImageLeftRaw());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_left_raw;
      // from cv_bridge to sensor_msgs::Image
      image_left_raw_bridge.toImageMsg(out_image_left_raw);
      // Publish the data
      publisher_left_raw_->publish(out_image_left_raw);
    }

    if (!cvgroup.getImageRightRaw().empty()) {
      // Convert to ROS data type
      cv_bridge::CvImage image_right_raw_bridge =
        cv_bridge::CvImage(
        last_right_raw_->header, sensor_msgs::image_encodings::BGR8,
          cvgroup.getImageRightRaw());
      // >> message to be sent
      sensor_msgs::msg::Image out_image_right_raw;
      // from cv_bridge to sensor_msgs::Image
      image_right_raw_bridge.toImageMsg(out_image_right_raw);
      // Publish the data
      publisher_right_raw_->publish(out_image_right_raw);
    }

    if (!cvgroup.getPointCloud().empty()) {
      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 out_pointcloud;
      pcl::toROSMsg(cvgroup.getPointCloud(), out_pointcloud);
      out_pointcloud.header = last_pointcloud_->header;
      publisher_pointcloud_->publish(out_pointcloud);
    }
  }

  // Last received data
  sensor_msgs::msg::Image::UniquePtr last_rgb_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr last_depth_ = nullptr;
  stereo_msgs::msg::DisparityImage::UniquePtr last_disparity_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr last_left_rect_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr last_right_rect_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr last_left_raw_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr last_right_raw_ = nullptr;
  sensor_msgs::msg::PointCloud2::UniquePtr last_pointcloud_ = nullptr;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_ = nullptr;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_rgb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr subscription_disparity_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_left_rect_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_right_rect_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_left_raw_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_right_raw_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_disparity_; //Publish Image instead of DisparityImage
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_rect_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_rect_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  //Counters
  size_t counter_rgb_ = 0;
  size_t counter_depth_ = 0;
  size_t counter_disparity_ = 0;
  size_t counter_left_rect_ = 0;
  size_t counter_right_rect_ = 0;
  size_t counter_left_raw_ = 0;
  size_t counter_right_raw_ = 0;
  size_t counter_pointcloud_ = 0;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__CVSUBSCRIBER_HPP_

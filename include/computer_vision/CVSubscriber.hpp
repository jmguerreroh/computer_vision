/*
 Copyright (c) 2023 José Miguel Guerrero Hernández

 Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     https://creativecommons.org/licenses/by-sa/4.0/

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

#ifndef INCLUDE_COMPUTER_VISION__CVSUBSCRIBER_HPP_
#define INCLUDE_COMPUTER_VISION__CVSUBSCRIBER_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace computer_vision
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class CVGroup
{
public:
  CVGroup(cv::Mat image_rgb, cv::Mat image_depth, pcl::PointCloud<pcl::PointXYZRGB> pointcloud)
  {
    image_rgb_ = image_rgb;
    image_depth_ = image_depth;
    pointcloud_ = pointcloud;
  }
  cv::Mat getImageRGB() {return image_rgb_;}
  cv::Mat getImageDepth() {return image_depth_;}
  pcl::PointCloud<pcl::PointXYZRGB> getPointCloud() {return pointcloud_;}

private:
  cv::Mat image_rgb_;
  cv::Mat image_depth_;
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
      std::bind(&CVSubscriber::topic_callback_info, this, _1));

    subscription_rgb_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_rgb_in", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
    subscription_depth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/image_depth_in", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
    subscription_pointcloud_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      this, "/pointcloud_in", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(10), *subscription_rgb_, *subscription_depth_, *subscription_pointcloud_);
    sync_->registerCallback(
      std::bind(
        &CVSubscriber::topic_callback_multi, this, _1, _2, _3));

    publisher_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_depth",
      rclcpp::SensorDataQoS().reliable());

    publisher_rgb_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_rgb",
      rclcpp::SensorDataQoS().reliable());

    publisher_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud",
      rclcpp::SensorDataQoS().reliable());
  }

private:
  CVGroup image_processing_multi(
    const cv::Mat in_image_rgb,
    const cv::Mat in_image_depth,
    const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
  const;

  void topic_callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    RCLCPP_INFO(get_logger(), "Camera info received");

    camera_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
    camera_model_->fromCameraInfo(*msg);

    subscription_info_ = nullptr;
  }

  void topic_callback_multi(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_depth_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg) const
  {
    // Check if camera model has been received
    if (camera_model_ == nullptr) {
      RCLCPP_WARN(get_logger(), "Camera Model not yet available");
      return;
    }

    // Check if depth image has been received
    if (image_depth_msg->encoding != "16UC1" && image_depth_msg->encoding != "32FC1") {
      RCLCPP_ERROR(get_logger(), "The image type has not depth info");
      return;
    }

    // Remove this condition if you want to process it anyway
    if ((publisher_rgb_->get_subscription_count() > 0 ) &&
      (publisher_depth_->get_subscription_count() > 0) &&
      (publisher_pointcloud_->get_subscription_count() > 0))
    {
      // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
      cv_bridge::CvImagePtr image_rgb_ptr, image_depth_ptr;
      try {
        image_rgb_ptr = cv_bridge::toCvCopy(*image_rgb_msg, sensor_msgs::image_encodings::BGR8);
        image_depth_ptr = cv_bridge::toCvCopy(
          *image_depth_msg,
          sensor_msgs::image_encodings::TYPE_32FC1);
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      cv::Mat image_rgb_raw = image_rgb_ptr->image;
      cv::Mat image_depth_raw = image_depth_ptr->image;

      // Convert to PCL data type
      pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
      pcl::fromROSMsg(*pointcloud_msg, pointcloud);

      // Image and PointCloud processing
      CVGroup cvgroup = image_processing_multi(
        image_rgb_raw, image_depth_raw, pointcloud);

      // Convert OpenCV Image to ROS Image
      cv_bridge::CvImage image_rgb_bridge =
        cv_bridge::CvImage(
        image_rgb_msg->header, sensor_msgs::image_encodings::BGR8,
        cvgroup.getImageRGB());

      cv_bridge::CvImage image_depth_bridge =
        cv_bridge::CvImage(
        image_depth_msg->header, sensor_msgs::image_encodings::TYPE_32FC1,
        cvgroup.getImageDepth());

      // >> message to be sent
      sensor_msgs::msg::Image out_image_rgb, out_image_depth;

      // from cv_bridge to sensor_msgs::Image
      image_rgb_bridge.toImageMsg(out_image_rgb);
      image_depth_bridge.toImageMsg(out_image_depth);

      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 out_pointcloud;
      pcl::toROSMsg(cvgroup.getPointCloud(), out_pointcloud);
      out_pointcloud.header = pointcloud_msg->header;

      // Publish the data
      publisher_rgb_->publish(out_image_rgb);
      publisher_depth_->publish(out_image_depth);
      publisher_pointcloud_->publish(out_pointcloud);
    }
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscription_rgb_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscription_depth_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
  subscription_pointcloud_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;
};

} // namespace computer_vision

#endif  // INCLUDE_COMPUTER_VISION__CVSUBSCRIBER_HPP_

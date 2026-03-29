/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", queue_size,
                          &ContinuousDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("/apriltag/AprilTagDetectionArray/measurements", 1);
      
  // 添加像素坐标发布器
  target_pixel_publisher_ = 
      nh.advertise<geometry_msgs::Point>("/apriltag/Point/pixel", 1);
      
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  refresh_params_service_ =
      pnh.advertiseService("refresh_tag_params", 
                          &ContinuousDetector::refreshParamsCallback, this);
}

void ContinuousDetector::refreshTagParameters()
{
  // Resetting the tag detector will cause a new param server lookup
  // So if the parameters have changed (by someone/something), 
  // they will be updated dynamically
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  ros::NodeHandle& pnh = getPrivateNodeHandle();
  tag_detector_.reset(new TagDetector(pnh));
}

bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request& req,
                                               std_srvs::Empty::Response& res)
{
  refreshTagParameters();
  return true;
}

geometry_msgs::Point ContinuousDetector::projectToPixel(
    const geometry_msgs::PoseWithCovarianceStamped& pose,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  geometry_msgs::Point pixel_point;
  
  // 提取3D位置
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double z = pose.pose.pose.position.z;
  
  // 检查z坐标，避免除零
  if (std::abs(z) < 1e-6) {
    ROS_WARN("Target z-coordinate is too close to zero, cannot project to pixel");
    pixel_point.x = -1;
    pixel_point.y = -1;
    pixel_point.z = 0;
    return pixel_point;
  }
  
  // 相机内参矩阵
  double fx = camera_info->K[0];  // K[0]
  double fy = camera_info->K[4];  // K[4]
  double cx = camera_info->K[2];  // K[2]
  double cy = camera_info->K[5];  // K[5]
  
  // 3D到像素坐标的投影
  // u = fx * (x/z) + cx
  // v = fy * (y/z) + cy
  double u = fx * (x / z) + cx;
  double v = fy * (y / z) + cy;
  
  pixel_point.x = u;
  pixel_point.y = v;
  pixel_point.z = z;  // 保存深度信息
  
  return pixel_point;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      target_pixel_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // 检测AprilTag并获取结果
  AprilTagDetectionArray detections = tag_detector_->detectTags(cv_image_, camera_info);
  
  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_.publish(detections);

  // 转换并发布像素坐标
  if (target_pixel_publisher_.getNumSubscribers() > 0 && !detections.detections.empty())
  {
    // 使用第一个检测到的tag
    const auto& detection = detections.detections[0];
    geometry_msgs::Point pixel_point = projectToPixel(detection.pose, camera_info);
    
    // 检查投影是否有效（在图像范围内）
    if (pixel_point.x >= 0 && pixel_point.x < camera_info->width &&
        pixel_point.y >= 0 && pixel_point.y < camera_info->height)
    {
      target_pixel_publisher_.publish(pixel_point);
      
      ROS_DEBUG("Published pixel coordinates: u=%.2f, v=%.2f, depth=%.3f", 
                pixel_point.x, pixel_point.y, pixel_point.z);
    }
    else
    {
      ROS_WARN("Projected pixel coordinates are outside image bounds: u=%.2f, v=%.2f", 
               pixel_point.x, pixel_point.y);
    }
  }

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
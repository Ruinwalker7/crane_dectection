// Created by Chengfu Zou on 2023.7.1

#ifndef RM_CAMERA_DRIVER_DAHENG_CAMERA_HPP_
#define RM_CAMERA_DRIVER_DAHENG_CAMERA_HPP_

// std
#include <memory>
// ROS
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// Daheng Galaxy Driver
#include "daheng/DxImageProc.h"
#include "daheng/GxIAPI.h"

namespace fyt::camera_driver
{
class DahengCameraNode : public rclcpp::Node
{
public:
  explicit DahengCameraNode(const rclcpp::NodeOptions & options);

  ~DahengCameraNode();

  bool open();
  void close();

  rclcpp::Time getLatestFrameStamp();

private:

  // Watch dog timer
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;

  // onFrameCallback
  static void GX_STDC onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM * pFrame);
  static rclcpp::Clock::SharedPtr node_clock;
  
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::msg::Image image_msg_;
  static sensor_msgs::msg::CameraInfo camera_info_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_;

  // Daheng Galaxy API
  static GX_DEV_HANDLE m_hDevice;
  static unsigned char * m_pBufferRaw;
  static GX_STATUS emStatus;
  static int64_t m_nPixelformat;
  static int64_t m_nPayLoadSize;
  static int64_t m_nBayerType;
  static int64_t MAX_RESOLUTION_WIDTH;
  static int64_t MAX_RESOLUTION_HEIGHT;

  // General
  bool is_open_ = false;
  int resolution_width_;
  int resolution_height_;
  int auto_white_balance_;
  int frame_rate_;
  int exposure_time_;
  double gain_;
  int nOffsetX;
  int nOffsetY;
};

}  // namespace fyt::camera_driver

#endif  // endif RM_CAMERA_DRIVER_DAHENG_CAMERA_HPP_
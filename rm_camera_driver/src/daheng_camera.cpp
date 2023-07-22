// Created by Chengfu Zou on 2023.7.1

#include "rm_camera_driver/daheng_camera.hpp"

#include <chrono>
#include <ctime>
#include <rclcpp/clock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "daheng/GxIAPI.h"

namespace fyt::camera_driver
{
DahengCameraNode::DahengCameraNode(const rclcpp::NodeOptions & options)
: Node("camera_driver", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Daheng Galaxy Camera Driver!");

  camera_name_ = this->declare_parameter("camera_name", "daheng");
  camera_info_url_ = this->declare_parameter(
    "camera_info_url", "package://rm_bringup/config/camera_info.yaml");
  frame_id_ = this->declare_parameter("camera_frame_id", "camera_optical_frame");
  pixel_format_ = this->declare_parameter("pixel_format", "rgb8");
  resolution_width_ = this->declare_parameter("resolution_width", 1280);
  resolution_height_ = this->declare_parameter("resolution_height", 1024);
  auto_white_balance_ = this->declare_parameter("auto_white_balance", 1);
  frame_rate_ = this->declare_parameter("frame_rate", 210);
  RCLCPP_INFO(this->get_logger(), "1 Daheng Galaxy Camera Driver!");
  exposure_time_ = this->declare_parameter("exposure_time", 2000);

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.floating_point_range.resize(1);
  param_desc.floating_point_range[0].from_value = 0.0;
  param_desc.floating_point_range[0].to_value = 16.0;
  param_desc.floating_point_range[0].step = 0.1;
  gain_ = this->declare_parameter("gain", 5.0);
  nOffsetX = this->declare_parameter("offsetX", 0);
  nOffsetY = this->declare_parameter("offsetY", 0);

  //为存储原始图像数据申请空间
  image_msg_.header.frame_id = frame_id_;
  image_msg_.encoding = pixel_format_;
  image_msg_.height = resolution_height_;
  image_msg_.width = resolution_width_;
  image_msg_.step = resolution_width_ * 3;
  image_msg_.data.resize(image_msg_.height * image_msg_.step);
  m_pBufferRaw = new unsigned char[(size_t)(image_msg_.height * image_msg_.step)];

  if (pixel_format_ == "mono8") {
    m_nPixelformat = GX_PIXEL_FORMAT_MONO8;
  } else if (pixel_format_ == "mono16") {
    m_nPixelformat = GX_PIXEL_FORMAT_MONO16;
  } else if (pixel_format_ == "bgr8") {
    m_nPixelformat = GX_PIXEL_FORMAT_BGR8;
    m_nBayerType = BAYERBG;
  } else if (pixel_format_ == "rgb8") {
    m_nPixelformat = GX_PIXEL_FORMAT_RGB8;
    m_nBayerType = BAYERRG;
  } else if (pixel_format_ == "bgra8") {
    m_nPixelformat = GX_PIXEL_FORMAT_BGRA8;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format!");
  }

  node_clock = this->get_clock();

  pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_img", 1);

  // Check if camera is alive every 100ms.
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&DahengCameraNode::timerCallback, this));
}

DahengCameraNode::~DahengCameraNode()
{
  close();
  if (m_pBufferRaw != NULL) {
    delete[] m_pBufferRaw;
  }
}

void DahengCameraNode::timerCallback()
{
  while (!is_open_ && rclcpp::ok()) {
    this->open();
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // set camera exposure
  int temp_exposure_time = this->get_parameter("exposure_time").as_int();
  double temp_gain = this->get_parameter("gain").as_double();
  
  if (abs(temp_exposure_time - exposure_time_) > 0.5 || abs(temp_gain - gain_) > 0.05) {
    exposure_time_ = temp_exposure_time;
    gain_ = temp_gain;
    GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, static_cast<double>(exposure_time_));
    GXSetFloat(m_hDevice, GX_FLOAT_GAIN, gain_);
    RCLCPP_INFO(this->get_logger(), "Set exposure_time: %d, gain: %f", exposure_time_, gain_);
  }
}


void DahengCameraNode::close()
{
  RCLCPP_INFO(this->get_logger(), "Closing Daheng Galaxy Camera Device!");
  if (is_open_) {
    //发送停采命令
    GXStreamOff(m_hDevice);
    //注销采集回调
    GXUnregisterCaptureCallback(m_hDevice);
    GXCloseDevice(m_hDevice);
  }
  GXCloseLib();
  is_open_ = false;
}

bool DahengCameraNode::open()
{
  RCLCPP_INFO(this->get_logger(), "Opening Daheng Galaxy Camera Device!");
  if (is_open_) {
    RCLCPP_WARN(
      this->get_logger(), "Daheng Galaxy Camera Device is already opened!, try to restart!");
    close();
  }
  emStatus = GX_STATUS_SUCCESS;
  GX_OPEN_PARAM openParam;
  uint32_t nDeviceNum = 0;
  openParam.accessMode = GX_ACCESS_EXCLUSIVE;
  openParam.openMode = GX_OPEN_INDEX;
  openParam.pszContent = (char *)"1";
  // 初始化库
  emStatus = GXInitLib();
  if (emStatus != GX_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Can't init lib");
    return false;
  }
  // 枚举设备列表
  emStatus = GXUpdateDeviceList(&nDeviceNum, 1000);
  if ((emStatus != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)) {
    RCLCPP_ERROR(this->get_logger(), "Can't find camera");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Found %d devices", nDeviceNum);
  //打开设备
  emStatus = GXOpenDevice(&openParam, &m_hDevice);
  if (emStatus != GX_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Can't open device");
    return false;
  }
  is_open_ = true;

  GXGetInt(m_hDevice, GX_INT_WIDTH_MAX, &MAX_RESOLUTION_WIDTH);
  GXGetInt(m_hDevice, GX_INT_HEIGHT_MAX, &MAX_RESOLUTION_HEIGHT);

  // 设置像素格式
  GXSetEnum(m_hDevice, GX_ENUM_PIXEL_FORMAT, m_nPixelformat);
  // 设置宽度
  GXSetInt(m_hDevice, GX_INT_WIDTH, resolution_width_);
  // 设置高度
  GXSetInt(m_hDevice, GX_INT_HEIGHT, resolution_height_);

  // 从中心裁剪
  GXSetInt(m_hDevice, GX_INT_OFFSET_X, nOffsetX);
  GXSetInt(m_hDevice, GX_INT_OFFSET_Y, nOffsetY);

  // 获取图像大小
  GXGetInt(m_hDevice, GX_INT_PAYLOAD_SIZE, &m_nPayLoadSize);
  //设置是否开启自动白平衡
  if (auto_white_balance_) {
    GXSetEnum(m_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, 1);
  } else {
    //设置白平衡数值  如果开启自动白平衡则无效
    GXSetEnum(m_hDevice, GX_ENUM_LIGHT_SOURCE_PRESET, GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K);
  }
  //设置帧率
  GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ENUM_COVER_FRAMESTORE_MODE_ON);
  GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate_);

  //设置曝光时间
  GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time_);
  //设置增益
  GXSetFloat(m_hDevice, GX_FLOAT_GAIN, gain_);

  //设置采集模式连续采集
  GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);

  //注册图像处理回调函数
  emStatus = GXRegisterCaptureCallback(m_hDevice, nullptr, onFrameCallbackFun);
  if (emStatus != GX_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Register capture callback function failed!");
    return false;
  }
  //发送开采命令
  emStatus = GXStreamOn(m_hDevice);
  if (emStatus != GX_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Send start acquisition command failed!");
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Daheng Galaxy Camera Device Open Success!");
  return true;
}

void GX_STDC DahengCameraNode::onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM * pFrame)
{
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
    // RGB转换
    DxRaw8toRGB24(
      (void *)pFrame->pImgBuf, m_pBufferRaw, pFrame->nWidth, pFrame->nHeight, RAW2RGB_NEIGHBOUR,
      static_cast<DX_PIXEL_COLOR_FILTER>(m_nBayerType), false);

    image_msg_.header.stamp = node_clock->now();
    memcpy(
      (unsigned char *)(&image_msg_.data[0]), m_pBufferRaw, image_msg_.step * image_msg_.height);
    pub_->publish(image_msg_);
  }
}

// Define static members

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr DahengCameraNode::pub_;
sensor_msgs::msg::Image DahengCameraNode::image_msg_;
sensor_msgs::msg::CameraInfo DahengCameraNode::camera_info_;
GX_DEV_HANDLE DahengCameraNode::m_hDevice;
unsigned char * DahengCameraNode::m_pBufferRaw;
GX_STATUS DahengCameraNode::emStatus;
int64_t DahengCameraNode::m_nPixelformat;
int64_t DahengCameraNode::m_nPayLoadSize;
int64_t DahengCameraNode::m_nBayerType;
int64_t DahengCameraNode::MAX_RESOLUTION_WIDTH = 1280;
int64_t DahengCameraNode::MAX_RESOLUTION_HEIGHT = 1024;
rclcpp::Clock::SharedPtr DahengCameraNode::node_clock;
}  // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::DahengCameraNode)

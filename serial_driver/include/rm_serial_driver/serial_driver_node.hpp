// Created by 邹承甫 on 2023.7.1

#ifndef SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
#define SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_

// std
#include <memory>
#include <rclcpp/node.hpp>
#include <thread>
// 3rd-party
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
// #include "interfaces/msg/gimbal_cmd.hpp"
// #include "interfaces/msg/serial_receive_data.hpp"
// #include "interfaces/srv/set_mode.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol_factory.hpp"
#include "rm_serial_driver/transporter_interface.hpp"

// Node wrapper for SerialDriver
// Implementing secondary development through the Protocol class
class SerialDriverNode : public rclcpp::Node
{
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions & options);

public:
  void listenLoop();

private:
  void setMode(const uint8_t mode);

private:
  std::unique_ptr<std::thread> listen_thread_;
  // protocol
  std::unique_ptr<Protocol> protocol_;

  // sub
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  // pub
  rclcpp::Publisher<interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_pub_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool initial_set_param_ = false;
  uint8_t previous_receive_mode_ = 0;
};


#endif  // SERIAL_DRIVER_SERIAL_DRIVER_NODE_HPP_
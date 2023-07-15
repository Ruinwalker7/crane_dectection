// Created by 邹承甫 on 2023.7.1

#include "rm_serial_driver/serial_driver_node.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/qos.hpp>
#include <thread>

#include "interfaces/msg/serial.hpp"
#include "interfaces/msg/serial_receive_data.hpp"
#include "rm_serial_driver/uart_transporter.hpp"

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions & options)
: Node("serial_driver", options)
{
  RCLCPP_INFO(get_logger(), "Starting SerialDriverNode!");
  // init
  std::string port_name = this->declare_parameter("port_name", "/dev/ttyUSB0");
  std::string protocol_type = this->declare_parameter("protocol", "infantry");

  protocol_ = ProtocolFactory::createProtocol(protocol_type, port_name);

  if (protocol_ == nullptr) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create protocol! Invalid protocol type: %s", protocol_type.c_str());
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Protocol Type : \"%s\"", protocol_type.c_str());

  // sub
  subscriptions_ = protocol_->get_subscriptions(this);
  for (auto sub : subscriptions_) {
    RCLCPP_INFO(this->get_logger(), "Subscribe to topic: \"%s\"", sub->get_topic_name());
  }
  // pub
  serial_receive_data_pub_ =
    this->create_publisher<interfaces::msg::SerialReceiveData>("/serial/receive", 10);

  // task thread
  listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
}

void SerialDriverNode::listenLoop()
{
  interfaces::msg::SerialReceiveData receive_data;
  while (rclcpp::ok()) {
    if (protocol_->receive(receive_data)) {
      serial_receive_data_pub_->publish(receive_data);

      if (!initial_set_param_ || receive_data.ctrl != previous_receive_mode_) {
        setMode(receive_data.ctrl);
      }
    } else {
      RCLCPP_WARN(
        get_logger(), "Failed to receive packet! error_message: %s",
        protocol_->get_error_message().c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(SerialDriverNode)
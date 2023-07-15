// Created by 邹承甫 on 2023.7.6
#ifndef SERIAL_DRIVER_PROTOCOL_HPP_
#define SERIAL_DRIVER_PROTOCOL_HPP_
// clang-format off
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "interfaces/msg/serial.hpp"
#include "interfaces/msg/serial_receive_data.hpp"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/uart_transporter.hpp"
// clang-format on

// 由于不同兵种存在不同的串口协议，如步兵和哨兵使用不同的协议
// 为了实现同一个同一个串口节点能够支持不同的协议，使用了工厂模式统一接口
// 需要扩展新的协议时，只需要在继承Protocol的类并实现对应的协议即可
// 写的比较丑陋，但能力有限，暂时没想到更好的方法
namespace protocol
{
typedef enum : unsigned char { Fire = 0x01, NotFire = 0x00 } FireState;

// Protocol interface
class Protocol
{
public:
  virtual ~Protocol() = default;

  // Send gimbal command
  virtual void send(const interfaces::msg::Serial & data) = 0;

  // Receive data from serial port
  virtual bool receive(interfaces::msg::SerialReceiveData & data) = 0;

  // Create subscriptions for SerialDriverNode, sorry for using raw pointer
  virtual std::vector<rclcpp::SubscriptionBase::SharedPtr> get_subscriptions(
    rclcpp::Node * node) = 0;

  virtual std::string get_error_message() = 0;

private:
};

class ProtocolNormal : public Protocol
{
public:
  explicit ProtocolNormal(std::string_view port_name);

  ~ProtocolNormal() = default;

  void send(const interfaces::msg::Serial & data) override;

  bool receive(interfaces::msg::SerialReceiveData & data) override;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> get_subscriptions(rclcpp::Node * node) override;

  std::string get_error_message() override { return packet_tool_->get_error_message(); }

private:
  FixedPacketTool<9>::SharedPtr packet_tool_;
};

}  // namespace protocol
#endif  // SERIAL_DRIVER_PROTOCOLS_HPP_
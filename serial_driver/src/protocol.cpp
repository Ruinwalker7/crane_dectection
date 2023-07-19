#include "rm_serial_driver/protocol.hpp"

#include <rclcpp/subscription_base.hpp>

namespace protocol
{
ProtocolNormal::ProtocolNormal(std::string_view port_name)
{
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<10>>(uart_transporter);
}

void ProtocolNormal::send(const interfaces::msg::Serial & data)
{
  FixedPacket<10> packet;
  packet.load_data<int8_t>(data.object1, 1);
  packet.load_data<int8_t>(data.object2, 2);
  packet.load_data<int8_t>(data.object3, 3);
  packet.load_data<int8_t>(data.object4, 4);
  packet.load_data<int8_t>(data.object5, 5);
  packet.load_data<int8_t>(data.object6, 6);
  packet.load_data<int8_t>(data.type, 7);
  packet.load_data<int8_t>(data.more, 8);
  packet_tool_->send_packet(packet);
}

bool ProtocolNormal::receive(interfaces::msg::SerialReceiveData & data)
{
  FixedPacket<10> packet;
  if (packet_tool_->recv_packet(packet)) {
    packet.unload_data(data.ctrl, 1);
    return true;
  } else {
    return false;
  }
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolNormal::get_subscriptions(
  rclcpp::Node * node)
{
  return {node->create_subscription<interfaces::msg::Serial>(
    "/type", rclcpp::SensorDataQoS(),
    [this](const interfaces::msg::Serial::SharedPtr msg) { this->send(*msg); })};
}

}  // namespace protocol

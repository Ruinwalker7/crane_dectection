// Created by 邹承甫 on 2023.7.6
#ifndef SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
#define SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_

#include <memory>
#include <string_view>

#include "rm_serial_driver/protocol.hpp"

using protocol::Protocol;

class ProtocolFactory
{
public:
  ProtocolFactory() = delete;
  // Factory method to create a protocol
  static std::unique_ptr<Protocol> createProtocol(
    std::string_view protocol_type, std::string_view port_name)
  {
    if (protocol_type == "infantry") {
      return std::make_unique<protocol::ProtocolNormal>(port_name);
    }
    if (protocol_type == "hero") {
      return std::make_unique<protocol::ProtocolNormal>(port_name);
    }
    if (protocol_type == "air") {
      return std::make_unique<protocol::ProtocolNormal>(port_name);
    }

    return nullptr;
  }
};

#endif  // PROTOCOL_FACTORY_HPP_
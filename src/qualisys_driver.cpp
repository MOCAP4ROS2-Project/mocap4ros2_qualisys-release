// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "mocap4r2_qualisys_driver/qualisys_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mocap4r2_qualisys_driver
{

namespace
{

bool parse_axis(const std::string & s, UpAxis & out)
{
  static const std::map<std::string, UpAxis> axes = {
    {"x", UpAxis::X_POS}, {"-x", UpAxis::X_NEG}, {"y", UpAxis::Y_POS},
    {"-y", UpAxis::Y_NEG}, {"z", UpAxis::Z_POS}, {"-z", UpAxis::Z_NEG}};
  const auto it = axes.find(s);
  if (it == axes.end()) {
    return false;
  }
  out = it->second;
  return true;
}

}  // namespace

QualisysDriver::QualisysDriver(const rclcpp::NodeOptions & options)
: QualisysDriver(options, &make_sdk_source)
{
}

QualisysDriver::QualisysDriver(const rclcpp::NodeOptions & options, SourceFactory factory)
: MocapSystemNode("qualisys_driver", "qualisys", options),
  factory_(std::move(factory))
{
  declare_parameter<std::string>("server_address", "127.0.0.1");
  declare_parameter<int>("base_port", 22222);
  declare_parameter<int>("udp_port", 0);
  declare_parameter<double>("timeout", 2.0);
  declare_parameter<std::string>("up_axis", "auto");
  declare_parameter<double>("latency", 0.0);
  mocap4r2_control::declare_frame_publisher_parameters(*this);
}

QualisysDriver::~QualisysDriver()
{
  stop_thread();
  source_.reset();
}

QualisysDriver::CallbackReturn
QualisysDriver::on_system_configure()
{
  QualisysConfig config;
  config.server_address = get_parameter("server_address").as_string();
  const auto base = get_parameter("base_port").as_int();
  const auto udp = get_parameter("udp_port").as_int();
  const double timeout = get_parameter("timeout").as_double();
  latency_ = get_parameter("latency").as_double();
  up_axis_ = get_parameter("up_axis").as_string();
  UpAxis forced = UpAxis::Z_POS;
  if (config.server_address.empty() || base < 1 || base > 65535 || udp < 0 || udp > 65535 ||
    timeout <= 0.0 || latency_ < 0.0 || (up_axis_ != "auto" && !parse_axis(up_axis_, forced)))
  {
    RCLCPP_ERROR(
      get_logger(), "invalid parameters: check server_address, ports, timeout, latency >= 0 "
      "and up_axis (auto, x, -x, y, -y, z, -z)");
    return CallbackReturn::FAILURE;
  }
  config.base_port = static_cast<uint16_t>(base);
  config.udp_port = static_cast<uint16_t>(udp);
  config.timeout = std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000.0));

  source_ = factory_();
  source_->connect(config);  // throws
  const UpAxis up = up_axis_ == "auto" ? source_->up_axis() : forced;
  converter_ = QualisysConverter(up);
  frame_rate_ = source_->frequency();
  server_label_ = "QTM at " + config.server_address;
  RCLCPP_INFO(
    get_logger(), "connected to QTM at %s (%s, %.0f Hz)", config.server_address.c_str(),
    source_->version().c_str(), source_->frequency());
  publisher_ = std::make_unique<mocap4r2_control::FramePublisher>(*this);
  return CallbackReturn::SUCCESS;
}

QualisysDriver::CallbackReturn
QualisysDriver::on_system_activate()
{
  running_ = true;
  thread_ = std::thread(&QualisysDriver::acquisition_loop, this);
  return CallbackReturn::SUCCESS;
}

QualisysDriver::CallbackReturn
QualisysDriver::on_system_deactivate()
{
  stop_thread();
  return CallbackReturn::SUCCESS;
}

QualisysDriver::CallbackReturn
QualisysDriver::on_system_cleanup()
{
  stop_thread();
  if (source_) {
    source_->disconnect();
  }
  source_.reset();
  publisher_.reset();
  return CallbackReturn::SUCCESS;
}

void
QualisysDriver::on_system_shutdown()
{
  stop_thread();
  source_.reset();
}

void
QualisysDriver::stop_thread()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void
QualisysDriver::acquisition_loop()
{
  QualisysFrame frame;
  bool first = true;
  uint32_t last = 0;
  while (running_) {
    if (!source_->next_frame(frame, std::chrono::milliseconds(100))) {
      if (!source_->connected()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "connection to QTM lost");
      }
      continue;
    }
    const auto received = now();
    if (!first && frame.frame_number == last) {
      continue;
    }
    first = false;
    last = frame.frame_number;
    auto out = converter_.convert(frame);
    out.capture_time = received - rclcpp::Duration::from_seconds(latency_);
    publisher_->publish(std::move(out));
  }
}

}  // namespace mocap4r2_qualisys_driver

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_qualisys_driver::QualisysDriver)

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

#ifndef MOCAP4R2_QUALISYS_DRIVER__QUALISYS_DRIVER_HPP_
#define MOCAP4R2_QUALISYS_DRIVER__QUALISYS_DRIVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_qualisys_driver/qualisys_conversion.hpp"
#include "mocap4r2_qualisys_driver/qualisys_source.hpp"

namespace mocap4r2_qualisys_driver
{

// MOCAP4ROS2 driver of Qualisys systems (QTM) through the Qualisys C++ SDK
// (RT protocol), streaming all frames to a dedicated acquisition thread.
class QualisysDriver : public mocap4r2_control::MocapSystemNode
{
public:
  using SourceFactory = std::function<std::unique_ptr<QualisysSource>()>;

  explicit QualisysDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // For tests: a driver reading from another source.
  QualisysDriver(const rclcpp::NodeOptions & options, SourceFactory factory);
  ~QualisysDriver() override;

protected:
  CallbackReturn on_system_configure() override;
  CallbackReturn on_system_activate() override;
  CallbackReturn on_system_deactivate() override;
  CallbackReturn on_system_cleanup() override;
  void on_system_shutdown() override;
  double nominal_frame_rate() const override {return frame_rate_;}
  std::string server() const override {return server_label_;}
  bool connected() const override {return source_ && source_->connected();}

private:
  void acquisition_loop();
  void stop_thread();

  SourceFactory factory_;
  std::unique_ptr<QualisysSource> source_;
  std::unique_ptr<mocap4r2_control::FramePublisher> publisher_;
  QualisysConverter converter_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> frame_rate_{0.0};
  double latency_{0.0};
  std::string up_axis_{"auto"};
  std::string server_label_;
};

}  // namespace mocap4r2_qualisys_driver

#endif  // MOCAP4R2_QUALISYS_DRIVER__QUALISYS_DRIVER_HPP_

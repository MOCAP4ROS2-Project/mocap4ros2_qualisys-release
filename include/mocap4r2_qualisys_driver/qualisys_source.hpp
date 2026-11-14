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

#ifndef MOCAP4R2_QUALISYS_DRIVER__QUALISYS_SOURCE_HPP_
#define MOCAP4R2_QUALISYS_DRIVER__QUALISYS_SOURCE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace mocap4r2_qualisys_driver
{

// One frame from Qualisys Track Manager (QTM), in metres, in the QTM global
// frame (whose up axis is reported by up_axis()).
struct QualisysFrame
{
  struct Marker
  {
    std::string label;
    std::array<double, 3> position{};
    bool visible{true};
    double residual{-1.0};
  };
  struct UnlabeledMarker
  {
    uint32_t id{0};
    std::array<double, 3> position{};
  };
  struct Body
  {
    std::string name;
    bool valid{true};
    std::array<double, 3> position{};
    std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};  // column-major
    double residual{-1.0};
  };
  struct Segment
  {
    uint32_t id{0};
    std::string name;
    int32_t parent_id{-1};
    std::array<double, 3> position{};
    std::array<double, 4> rotation{0, 0, 0, 1};  // x, y, z, w (global)
  };
  struct Skeleton
  {
    std::string name;
    std::vector<Segment> segments;
  };

  uint32_t frame_number{0};
  uint64_t timestamp_us{0};
  std::vector<Marker> markers;
  std::vector<UnlabeledMarker> unlabeled_markers;
  std::vector<Body> bodies;
  std::vector<Skeleton> skeletons;
};

enum class UpAxis {X_POS, X_NEG, Y_POS, Y_NEG, Z_POS, Z_NEG};

struct QualisysConfig
{
  std::string server_address{"127.0.0.1"};
  uint16_t base_port{22222};
  uint16_t udp_port{0};  // 0: stream over TCP
  std::chrono::milliseconds timeout{2000};
};

class QualisysSource
{
public:
  virtual ~QualisysSource() = default;
  // Connect, read the settings and start streaming. Throws std::runtime_error.
  virtual void connect(const QualisysConfig & config) = 0;
  virtual bool next_frame(QualisysFrame & frame, std::chrono::milliseconds timeout) = 0;
  virtual bool connected() const = 0;
  virtual void disconnect() = 0;
  virtual double frequency() const = 0;
  virtual UpAxis up_axis() const = 0;
  virtual std::string version() const = 0;
};

std::unique_ptr<QualisysSource> make_sdk_source();

}  // namespace mocap4r2_qualisys_driver

#endif  // MOCAP4R2_QUALISYS_DRIVER__QUALISYS_SOURCE_HPP_

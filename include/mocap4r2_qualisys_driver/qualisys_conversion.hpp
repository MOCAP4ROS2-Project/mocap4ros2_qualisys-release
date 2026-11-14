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

#ifndef MOCAP4R2_QUALISYS_DRIVER__QUALISYS_CONVERSION_HPP_
#define MOCAP4R2_QUALISYS_DRIVER__QUALISYS_CONVERSION_HPP_

#include <array>
#include <map>
#include <string>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_qualisys_driver/qualisys_source.hpp"

namespace mocap4r2_qualisys_driver
{

// Rotation taking the QTM up axis to +Z (row-major 3x3).
std::array<double, 9> up_to_z(UpAxis up);

// Quaternion (x, y, z, w) of a column-major rotation matrix (normalized).
std::array<double, 4> matrix_to_quaternion(const std::array<double, 9> & column_major);

// Converts QTM frames to MOCAP4ROS2 frames in a Z-up world, with stable ids
// for named markers and bodies.
class QualisysConverter
{
public:
  explicit QualisysConverter(UpAxis up = UpAxis::Z_POS);
  mocap4r2_control::MocapFrame convert(const QualisysFrame & frame);

private:
  int32_t id_of(std::map<std::string, int32_t> & ids, const std::string & name);
  std::array<double, 3> point(const std::array<double, 3> & p) const;
  std::array<double, 4> rotation(const std::array<double, 9> & column_major) const;

  std::array<double, 9> r_;  // row-major
  std::map<std::string, int32_t> body_ids_;
  std::map<std::string, int32_t> marker_ids_;
};

}  // namespace mocap4r2_qualisys_driver

#endif  // MOCAP4R2_QUALISYS_DRIVER__QUALISYS_CONVERSION_HPP_

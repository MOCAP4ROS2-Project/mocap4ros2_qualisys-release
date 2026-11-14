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

#include <cmath>
#include <map>
#include <string>

#include "mocap4r2_qualisys_driver/qualisys_conversion.hpp"

namespace mocap4r2_qualisys_driver
{

namespace
{

using M3 = std::array<double, 9>;  // row-major

M3 from_column_major(const std::array<double, 9> & c)
{
  return {c[0], c[3], c[6], c[1], c[4], c[7], c[2], c[5], c[8]};
}

M3 mul(const M3 & a, const M3 & b)
{
  M3 r{};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        r[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
      }
    }
  }
  return r;
}

M3 transpose(const M3 & a)
{
  return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

std::array<double, 4> quat_of(const M3 & m)
{
  // Shepperd's method, robust for all rotations.
  const double t = m[0] + m[4] + m[8];
  double x, y, z, w;
  if (t > 0.0) {
    const double s = std::sqrt(t + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m[7] - m[5]) / s;
    y = (m[2] - m[6]) / s;
    z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
    w = (m[7] - m[5]) / s;
    x = 0.25 * s;
    y = (m[1] + m[3]) / s;
    z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
    w = (m[2] - m[6]) / s;
    x = (m[1] + m[3]) / s;
    y = 0.25 * s;
    z = (m[5] + m[7]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
    w = (m[3] - m[1]) / s;
    x = (m[2] + m[6]) / s;
    y = (m[5] + m[7]) / s;
    z = 0.25 * s;
  }
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (!(n > 1e-9) || !std::isfinite(n)) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {x / n, y / n, z / n, w / n};
}

M3 matrix_of(const std::array<double, 4> & q)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
}

geometry_msgs::msg::Pose pose(const std::array<double, 3> & p, const std::array<double, 4> & q)
{
  geometry_msgs::msg::Pose m;
  m.position.x = p[0];
  m.position.y = p[1];
  m.position.z = p[2];
  m.orientation.x = q[0];
  m.orientation.y = q[1];
  m.orientation.z = q[2];
  m.orientation.w = q[3];
  return m;
}

bool finite3(const std::array<double, 3> & p)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

}  // namespace

std::array<double, 9>
up_to_z(UpAxis up)
{
  switch (up) {
    case UpAxis::X_POS: return {0, 0, -1, 0, 1, 0, 1, 0, 0};
    case UpAxis::X_NEG: return {0, 0, 1, 0, 1, 0, -1, 0, 0};
    case UpAxis::Y_POS: return {1, 0, 0, 0, 0, -1, 0, 1, 0};
    case UpAxis::Y_NEG: return {1, 0, 0, 0, 0, 1, 0, -1, 0};
    case UpAxis::Z_NEG: return {1, 0, 0, 0, -1, 0, 0, 0, -1};
    case UpAxis::Z_POS:
    default: return {1, 0, 0, 0, 1, 0, 0, 0, 1};
  }
}

std::array<double, 4>
matrix_to_quaternion(const std::array<double, 9> & column_major)
{
  return quat_of(from_column_major(column_major));
}

QualisysConverter::QualisysConverter(UpAxis up)
: r_(up_to_z(up))
{
}

int32_t
QualisysConverter::id_of(std::map<std::string, int32_t> & ids, const std::string & name)
{
  const auto it = ids.find(name);
  if (it != ids.end()) {
    return it->second;
  }
  const auto id = static_cast<int32_t>(ids.size() + 1);
  ids[name] = id;
  return id;
}

std::array<double, 3>
QualisysConverter::point(const std::array<double, 3> & p) const
{
  return {r_[0] * p[0] + r_[1] * p[1] + r_[2] * p[2],
    r_[3] * p[0] + r_[4] * p[1] + r_[5] * p[2],
    r_[6] * p[0] + r_[7] * p[1] + r_[8] * p[2]};
}

std::array<double, 4>
QualisysConverter::rotation(const std::array<double, 9> & column_major) const
{
  // Body axes keep their meaning: R_up * M * R_up^T.
  return quat_of(mul(mul(r_, from_column_major(column_major)), transpose(r_)));
}

mocap4r2_control::MocapFrame
QualisysConverter::convert(const QualisysFrame & frame)
{
  using mocap4r2_interfaces::msg::Marker;
  using mocap4r2_interfaces::msg::RigidBody;
  using mocap4r2_interfaces::msg::Segment;
  using mocap4r2_interfaces::msg::Skeleton;

  mocap4r2_control::MocapFrame out;
  out.frame_number = frame.frame_number;

  std::map<std::string, std::vector<Marker>> by_body;
  for (const auto & m : frame.markers) {
    Marker mk;
    mk.name = m.label;
    mk.id = m.label.empty() ? Marker::UNKNOWN_ID : id_of(marker_ids_, m.label);
    mk.occluded = !m.visible || !finite3(m.position);
    if (!mk.occluded) {
      const auto p = point(m.position);
      mk.position.x = p[0];
      mk.position.y = p[1];
      mk.position.z = p[2];
    }
    mk.residual = static_cast<float>(m.residual);
    out.markers.push_back(mk);
    // QTM rigid body markers are usually named "<body>_<marker>" or "<body> - <marker>".
    for (const auto & b : frame.bodies) {
      if (!b.name.empty() && m.label.rfind(b.name, 0) == 0) {
        by_body[b.name].push_back(mk);
        break;
      }
    }
  }
  for (const auto & u : frame.unlabeled_markers) {
    Marker mk;
    mk.id = Marker::UNKNOWN_ID;
    const auto p = point(u.position);
    mk.position.x = p[0];
    mk.position.y = p[1];
    mk.position.z = p[2];
    mk.residual = -1.0f;
    out.markers.push_back(mk);
  }

  for (const auto & b : frame.bodies) {
    RigidBody rb;
    rb.id = id_of(body_ids_, b.name);
    rb.name = b.name;
    rb.tracking_valid = b.valid && finite3(b.position);
    if (rb.tracking_valid) {
      rb.pose = pose(point(b.position), rotation(b.rotation));
    } else {
      rb.pose.orientation.w = 1.0;  // no NaN poses downstream
    }
    rb.mean_error = static_cast<float>(b.residual);
    const auto markers = by_body.find(b.name);
    if (markers != by_body.end()) {
      rb.markers = markers->second;
    }
    out.rigid_bodies.push_back(rb);
  }

  for (const auto & s : frame.skeletons) {
    Skeleton sk;
    sk.id = id_of(body_ids_, "skeleton:" + s.name);
    sk.name = s.name;
    sk.tracking_valid = !s.segments.empty();
    std::map<uint32_t, bool> ids;
    for (const auto & seg : s.segments) {
      ids[seg.id] = true;
    }
    for (const auto & seg : s.segments) {
      Segment out_seg;
      out_seg.id = static_cast<int32_t>(seg.id);
      out_seg.name = seg.name.empty() ? std::to_string(seg.id) : seg.name;
      out_seg.parent_id = seg.parent_id > 0 && ids.count(static_cast<uint32_t>(seg.parent_id)) &&
        static_cast<uint32_t>(seg.parent_id) != seg.id ? seg.parent_id : Segment::NO_PARENT;
      out_seg.tracking_valid = finite3(seg.position) && std::isfinite(seg.rotation[3]);
      if (out_seg.tracking_valid) {
        out_seg.pose = pose(
          point(seg.position), quat_of(mul(mul(r_, matrix_of(seg.rotation)), transpose(r_))));
      } else {
        out_seg.pose.orientation.w = 1.0;
      }
      sk.tracking_valid = sk.tracking_valid && out_seg.tracking_valid;
      sk.segments.push_back(out_seg);
    }
    out.skeletons.push_back(sk);
  }
  return out;
}

}  // namespace mocap4r2_qualisys_driver

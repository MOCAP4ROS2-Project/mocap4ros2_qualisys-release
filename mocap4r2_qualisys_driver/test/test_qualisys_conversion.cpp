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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "mocap4r2_qualisys_driver/qualisys_conversion.hpp"

using mocap4r2_interfaces::msg::Marker;
using mocap4r2_interfaces::msg::Segment;
using mocap4r2_qualisys_driver::matrix_to_quaternion;
using mocap4r2_qualisys_driver::QualisysConverter;
using mocap4r2_qualisys_driver::QualisysFrame;
using mocap4r2_qualisys_driver::up_to_z;
using mocap4r2_qualisys_driver::UpAxis;

namespace
{

// Column-major rotation of angle a about z.
std::array<double, 9> rot_z(double a)
{
  return {std::cos(a), std::sin(a), 0, -std::sin(a), std::cos(a), 0, 0, 0, 1};
}

QualisysFrame frame()
{
  QualisysFrame f;
  f.frame_number = 77;
  f.markers = {{"robot_front", {1, 2, 3}, true, 0.0005}, {"robot_back", {0, 0, 0}, false, -1.0},
    {"loose", {4, 4, 4}, true, 0.001}};
  f.unlabeled_markers = {{99, {7, 7, 7}}};
  QualisysFrame::Body b;
  b.name = "robot";
  b.position = {1, 2, 3};
  b.rotation = rot_z(0.5);
  b.residual = 0.0004;
  QualisysFrame::Body lost;
  lost.name = "wand";
  lost.valid = false;
  lost.position = {std::nan(""), std::nan(""), std::nan("")};
  f.bodies = {b, lost};
  return f;
}

}  // namespace

TEST(QualisysConversion, matrix_to_quaternion_cases)
{
  auto q = matrix_to_quaternion({1, 0, 0, 0, 1, 0, 0, 0, 1});
  EXPECT_DOUBLE_EQ(q[3], 1.0);
  q = matrix_to_quaternion(rot_z(M_PI_2));
  EXPECT_NEAR(q[2], std::sin(M_PI_4), 1e-12);
  EXPECT_NEAR(q[3], std::cos(M_PI_4), 1e-12);
  // 180 degrees about x (trace -1 branch)
  q = matrix_to_quaternion({1, 0, 0, 0, -1, 0, 0, 0, -1});
  EXPECT_NEAR(std::abs(q[0]), 1.0, 1e-12);
  // 180 about y and about z
  EXPECT_NEAR(std::abs(matrix_to_quaternion({-1, 0, 0, 0, 1, 0, 0, 0, -1})[1]), 1.0, 1e-12);
  EXPECT_NEAR(std::abs(matrix_to_quaternion({-1, 0, 0, 0, -1, 0, 0, 0, 1})[2]), 1.0, 1e-12);
  // NaN matrix: identity, never NaN downstream
  const double n = std::nan("");
  q = matrix_to_quaternion({n, n, n, n, n, n, n, n, n});
  EXPECT_DOUBLE_EQ(q[3], 1.0);
}

TEST(QualisysConversion, every_up_axis_maps_to_z)
{
  const std::pair<UpAxis, std::array<double, 3>> cases[] = {
    {UpAxis::X_POS, {1, 0, 0}}, {UpAxis::X_NEG, {-1, 0, 0}}, {UpAxis::Y_POS, {0, 1, 0}},
    {UpAxis::Y_NEG, {0, -1, 0}}, {UpAxis::Z_POS, {0, 0, 1}}, {UpAxis::Z_NEG, {0, 0, -1}}};
  for (const auto & [axis, up] : cases) {
    const auto r = up_to_z(axis);
    EXPECT_NEAR(r[6] * up[0] + r[7] * up[1] + r[8] * up[2], 1.0, 1e-12);
    // Proper rotation: determinant +1
    const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
      r[2] * (r[3] * r[7] - r[4] * r[6]);
    EXPECT_NEAR(det, 1.0, 1e-12);
  }
}

TEST(QualisysConversion, bodies_markers_and_validity)
{
  QualisysConverter c;
  const auto out = c.convert(frame());
  EXPECT_EQ(out.frame_number, 77u);
  ASSERT_EQ(out.rigid_bodies.size(), 2u);
  const auto & robot = out.rigid_bodies[0];
  EXPECT_TRUE(robot.tracking_valid);
  EXPECT_DOUBLE_EQ(robot.pose.position.y, 2.0);
  EXPECT_NEAR(robot.pose.orientation.z, std::sin(0.25), 1e-12);
  EXPECT_FLOAT_EQ(robot.mean_error, 0.0004f);
  ASSERT_EQ(robot.markers.size(), 2u);  // robot_front, robot_back
  EXPECT_TRUE(robot.markers[1].occluded);
  const auto & wand = out.rigid_bodies[1];
  EXPECT_FALSE(wand.tracking_valid);
  EXPECT_FALSE(std::isnan(wand.pose.position.x));  // never NaN poses
  EXPECT_DOUBLE_EQ(wand.pose.orientation.w, 1.0);

  ASSERT_EQ(out.markers.size(), 4u);
  EXPECT_EQ(out.markers[0].name, "robot_front");
  EXPECT_NE(out.markers[0].id, Marker::UNKNOWN_ID);
  EXPECT_EQ(out.markers[3].id, Marker::UNKNOWN_ID);  // unlabeled trajectory
  EXPECT_DOUBLE_EQ(out.markers[3].position.x, 7.0);
}

TEST(QualisysConversion, y_up_calibration)
{
  // QTM calibrated Y-up: a body 1 m up (Y) yawed about Y becomes 1 m up (Z) yawed about Z.
  QualisysConverter c(UpAxis::Y_POS);
  QualisysFrame f;
  QualisysFrame::Body b;
  b.name = "b";
  b.position = {2, 1, -3};
  const double a = 0.4;
  b.rotation = {std::cos(a), 0, -std::sin(a), 0, 1, 0, std::sin(a), 0, std::cos(a)};  // about y
  f.bodies = {b};
  const auto out = c.convert(f);
  const auto & p = out.rigid_bodies[0].pose;
  EXPECT_NEAR(p.position.x, 2.0, 1e-12);
  EXPECT_NEAR(p.position.y, 3.0, 1e-12);
  EXPECT_NEAR(p.position.z, 1.0, 1e-12);
  EXPECT_NEAR(p.orientation.x, 0.0, 1e-12);
  EXPECT_NEAR(p.orientation.y, 0.0, 1e-12);
  EXPECT_NEAR(p.orientation.z, std::sin(a / 2), 1e-12);
}

TEST(QualisysConversion, ids_stable_and_skeletons)
{
  QualisysConverter c;
  auto f = frame();
  QualisysFrame::Skeleton sk;
  sk.name = "alice";
  sk.segments = {{1, "Hips", 0, {0, 0, 1}, {0, 0, 0, 1}},
    {2, "Spine", 1, {0, 0, 1.2}, {0, 0, 0, 1}},
    {3, "Bad", 3, {std::nan(""), 0, 0}, {0, 0, 0, 1}}};
  f.skeletons = {sk};
  const auto a = c.convert(f);
  std::swap(f.bodies[0], f.bodies[1]);
  const auto b = c.convert(f);
  EXPECT_EQ(a.rigid_bodies[0].id, b.rigid_bodies[1].id);
  ASSERT_EQ(a.skeletons.size(), 1u);
  const auto & s = a.skeletons[0];
  EXPECT_EQ(s.segments[0].parent_id, Segment::NO_PARENT);
  EXPECT_EQ(s.segments[1].parent_id, 1);
  EXPECT_EQ(s.segments[2].parent_id, Segment::NO_PARENT);  // self parent
  EXPECT_FALSE(s.segments[2].tracking_valid);
  EXPECT_FALSE(s.tracking_valid);
  EXPECT_NE(s.id, a.rigid_bodies[0].id);
}

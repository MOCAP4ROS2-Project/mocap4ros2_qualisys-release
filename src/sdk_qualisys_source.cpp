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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "RTProtocol.h"
#include "mocap4r2_qualisys_driver/qualisys_source.hpp"

namespace mocap4r2_qualisys_driver
{

namespace
{

constexpr double kMm = 0.001;

class SdkQualisysSource : public QualisysSource
{
public:
  ~SdkQualisysSource() override {disconnect();}

  void connect(const QualisysConfig & config) override
  {
    uint16_t udp = config.udp_port;
    if (!rtp_.Connect(
        config.server_address.c_str(), config.base_port, config.udp_port ? &udp : nullptr))
    {
      throw std::runtime_error(
              "cannot connect to QTM at " + config.server_address + ": " + rtp_.GetErrorString());
    }
    bool available = false;
    if (rtp_.ReadGeneralSettings()) {
      frequency_ = rtp_.GetSystemFrequency();
    }
    if (rtp_.Read3DSettings(available) && available) {
      std::string calibration;
      std::vector<CRTProtocol::SSettings3DLabel> labels;
      std::vector<CRTProtocol::SSettingsBone> bones;
      CRTProtocol::EAxis axis;
      rtp_.Get3DSettings(axis, calibration, labels, bones);
      up_ = static_cast<UpAxis>(static_cast<int>(axis));
    }
    rtp_.Read6DOFSettings(available);
    rtp_.ReadSkeletonSettings(available, true);
    for (unsigned int s = 0; s < rtp_.GetSkeletonCount(); s++) {
      CRTProtocol::SSettingsSkeleton sk;
      if (rtp_.GetSkeleton(s, &sk)) {
        skeleton_settings_.push_back(sk);
      }
    }
    // All frames, labelled and unlabelled markers with residuals, bodies, global skeletons.
    if (!rtp_.StreamFrames(
        CRTProtocol::EStreamRate::RateAllFrames, 0, udp, nullptr,
        "3DRes 3DNoLabels 6DRes Skeleton:global"))
    {
      throw std::runtime_error(std::string("QTM refused to stream: ") + rtp_.GetErrorString());
    }
  }

  bool next_frame(QualisysFrame & f, std::chrono::milliseconds timeout) override
  {
    CRTPacket::EPacketType type;
    const int r = rtp_.ReceiveRTPacket(type, true, static_cast<int>(timeout.count() * 1000));
    if (r <= 0 || type != CRTPacket::PacketData) {
      return false;
    }
    read(*rtp_.GetRTPacket(), f);
    return true;
  }

  bool connected() const override {return rtp_.Connected();}

  void disconnect() override
  {
    if (rtp_.Connected()) {
      rtp_.StreamFramesStop();
      rtp_.Disconnect();
    }
  }

  double frequency() const override {return frequency_;}
  UpAxis up_axis() const override {return up_;}
  std::string version() const override
  {
    return "RT protocol " + std::to_string(MAJOR_VERSION) +
           "." + std::to_string(MINOR_VERSION);
  }

private:
  static const char * safe(const char * s) {return s ? s : "";}

  void read(CRTPacket & p, QualisysFrame & f)
  {
    f = QualisysFrame();
    f.frame_number = p.GetFrameNumber();
    f.timestamp_us = p.GetTimeStamp();

    for (unsigned int i = 0; i < p.Get3DResidualMarkerCount(); i++) {
      float x, y, z, res;
      if (!p.Get3DResidualMarker(i, x, y, z, res)) {
        continue;
      }
      QualisysFrame::Marker m;
      m.label = safe(rtp_.Get3DLabelName(i));  // settings may lag behind QTM
      m.visible = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      m.position = {x * kMm, y * kMm, z * kMm};
      m.residual = std::isfinite(res) ? res * kMm : -1.0;
      f.markers.push_back(m);
    }
    for (unsigned int i = 0; i < p.Get3DNoLabelsMarkerCount(); i++) {
      float x, y, z;
      unsigned int id;
      if (p.Get3DNoLabelsMarker(i, x, y, z, id)) {
        f.unlabeled_markers.push_back({id, {x * kMm, y * kMm, z * kMm}});
      }
    }
    for (unsigned int i = 0; i < p.Get6DOFResidualBodyCount(); i++) {
      float x, y, z, rot[9], res;
      if (!p.Get6DOFResidualBody(i, x, y, z, rot, res)) {
        continue;
      }
      QualisysFrame::Body b;
      b.name = safe(rtp_.Get6DOFBodyName(i));
      if (b.name.empty()) {
        b.name = "body" + std::to_string(i);
      }
      b.valid = std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(rot[0]);
      b.position = {x * kMm, y * kMm, z * kMm};
      for (int k = 0; k < 9; k++) {
        b.rotation[k] = rot[k];
      }
      b.residual = std::isfinite(res) ? res * kMm : -1.0;
      f.bodies.push_back(b);
    }
    for (unsigned int s = 0; s < p.GetSkeletonCount(); s++) {
      QualisysFrame::Skeleton sk;
      const CRTProtocol::SSettingsSkeleton * settings =
        s < skeleton_settings_.size() ? &skeleton_settings_[s] : nullptr;
      sk.name = settings ? settings->name : "skeleton" + std::to_string(s);
      for (unsigned int g = 0; g < p.GetSkeletonSegmentCount(s); g++) {
        CRTPacket::SSkeletonSegment seg;
        if (!p.GetSkeletonSegment(s, g, seg)) {
          continue;
        }
        QualisysFrame::Segment out;
        out.id = seg.id;
        out.position = {seg.positionX * kMm, seg.positionY * kMm, seg.positionZ * kMm};
        out.rotation = {seg.rotationX, seg.rotationY, seg.rotationZ, seg.rotationW};
        if (settings) {
          for (const auto & d : settings->segments) {
            if (d.id == seg.id) {
              out.name = d.name;
              out.parent_id = d.parentId;
            }
          }
        }
        sk.segments.push_back(out);
      }
      f.skeletons.push_back(sk);
    }
  }

  CRTProtocol rtp_;
  double frequency_{0.0};
  UpAxis up_{UpAxis::Z_POS};
  std::vector<CRTProtocol::SSettingsSkeleton> skeleton_settings_;
};

}  // namespace

std::unique_ptr<QualisysSource>
make_sdk_source()
{
  return std::make_unique<SdkQualisysSource>();
}

}  // namespace mocap4r2_qualisys_driver

// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__ROBOT_TO_ROBOT_RTLS_PLUGIN_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__ROBOT_TO_ROBOT_RTLS_PLUGIN_HPP_

// std
#include <optional>
#include <string>
#include <vector>

// romea
#include "romea_core_localisation/observation_pose.hpp"
#include "romea_core_localisation_rtls/rtls_plugin_base.hpp"
#include "romea_core_rtls/trilateration/pose2D_estimator.hpp"

namespace romea {
namespace core {
namespace localisation {

class R2RRTLSPlugin : public RTLSPluginBase {
 public:
  R2RRTLSPlugin(
      const double& rangeStd, const double& minimalRange,
      const double& maximal_range, const uint8_t& rxPowerRejectionThreshold,
      const VectorOfEigenVector<Eigen::Vector3d>& initiatorsPositions,
      const VectorOfEigenVector<Eigen::Vector3d>& respondersPositions);

  virtual ~R2RRTLSPlugin() = default;

  bool compute_leader_pose(ObservationPose& leaserPose);

 private:
  void store_range2d(const size_t& initiatorIndex, const size_t& responderIndex,
                     const double& value) override;

  void reset_range2d(const size_t& initiatorIndex,
                     const size_t& responderIndex) override;

  bool estimate_leader_pose_();

 private:
  RTLSPose2DEstimator pose_estimator_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__ROBOT_TO_ROBOT_RTLS_PLUGIN_HPP_

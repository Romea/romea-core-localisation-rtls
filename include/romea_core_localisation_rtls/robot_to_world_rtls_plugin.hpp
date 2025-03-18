// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__ROBOT_TO_WORLD_RTLS_PLUGIN_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__ROBOT_TO_WORLD_RTLS_PLUGIN_HPP_

// std
#include <optional>
#include <vector>
#include <string>
#include <mutex>

// romea
#include "romea_core_rtls/trilateration/RTLSPose2DEstimator.hpp"
#include "romea_core_rtls/coordination/RTLSReachableTransceivers.hpp"
#include "romea_core_localisation_rtls/rtls_plugin_base.hpp"
#include "romea_core_localisation/observation_pose.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2WRTLSPlugin : public RTLSPluginBase
{
public:
  R2WRTLSPlugin(
    const double & rangeStd,
    const double & minimalRange,
    const double & maximalRange,
    const uint8_t & rxPowerRejectionThreshold,
    const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
    const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions);

  virtual ~R2WRTLSPlugin() = default;

  void select_responders_ranges(const std::vector<size_t> & respondersIndexes);

  bool compute_pose(ObservationPose & pose_observation);

private:
  void store_range2d(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const double & value) override;

  void reset_range2d(
    const size_t & initiatorIndex,
    const size_t & responderIndex) override;

  bool estimate_pose_();

private:
  RTLSReachableTransceivers reachable_responders_;
  RTLSPose2DEstimator pose_estimator_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__ROBOT_TO_WORLD_RTLS_PLUGIN_HPP_

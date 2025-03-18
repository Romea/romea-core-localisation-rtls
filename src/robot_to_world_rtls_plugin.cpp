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

// std
#include <optional>
#include <vector>
#include <string>

// romea
#include "romea_core_localisation_rtls/robot_to_world_rtls_plugin.hpp"

const double MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE = 20;

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WRTLSPlugin::R2WRTLSPlugin(
  const double & rangeStd,
  const double & minimalRange,
  const double & maximalRange,
  const uint8_t & rxPowerRejectionThreshold,
  const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
  const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions)
: RTLSPluginBase(
    rangeStd,
    minimalRange,
    maximalRange,
    rxPowerRejectionThreshold,
    initiatorsPositions,
    respondersPositions),
  reachable_responders_(respondersPositions, maximalRange),
  pose_estimator_(initiatorsPositions, respondersPositions)
{
  ranges2d_ = TrilaterationRangeBuffer(initiatorsPositions.size(), respondersPositions.size());
}

//-----------------------------------------------------------------------------
void R2WRTLSPlugin::select_responders_ranges(
  const std::vector<size_t> & respondersIndexes)
{
  for (size_t j = 0; j < responders_positions_.size(); ++j) {
    auto it = std::find(respondersIndexes.begin(), respondersIndexes.end(), j);
    if (it == respondersIndexes.end()) {
      for (size_t i = 0; i < initiators_positions_.size(); ++i) {
        reset_range2d(i, j);
      }
    }
  }
}

//-----------------------------------------------------------------------------
bool R2WRTLSPlugin::compute_pose(ObservationPose & pose_observation)
{
  if (estimate_pose_()) {
    pose_observation.firstMoment = pose_estimator_.getEstimate();
    pose_observation.secondMoment = pose_estimator_.getEstimateCovariance();
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool R2WRTLSPlugin::estimate_pose_()
{
  // std::cout << " i" << std::endl;
  // for (const auto & i :initiatorsPositions_) {
  //   std::cout << i.transpose() << std::endl;
  // }

  // std::cout << " r" << std::endl;
  // for (const auto & r :respondersPositions_) {
  //   std::cout << r.transpose() << std::endl;
  // }

  // std::cout << ranges2D_ << std::endl;
  return pose_estimator_.init(ranges2d_.data()) && pose_estimator_.estimate(
    MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, range_std_);
}

//-----------------------------------------------------------------------------
void R2WRTLSPlugin::store_range2d(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const double & value)
{
  ranges2d_.set(initiatorIndex, responderIndex, value);
}

//-----------------------------------------------------------------------------
void R2WRTLSPlugin::reset_range2d(
  const size_t & initiatorIndex,
  const size_t & responderIndex)
{
  ranges2d_.reset(initiatorIndex, responderIndex);
}

}   // namespace localisation
}   // namespace core
}   // namespace romea

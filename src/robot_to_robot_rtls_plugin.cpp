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

// std
#include <optional>
#include <string>
#include <vector>

// romea
#include "romea_core_localisation_rtls/robot_to_robot_rtls_plugin.hpp"

const double MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE = 20;

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2RRTLSPlugin::R2RRTLSPlugin(
  const double & rangeStd,
  const double & minimalRange,
  const double & maximal_range,
  const uint8_t & rxPowerRejectionThreshold,
  const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
  const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions)
: RTLSPluginBase(
    rangeStd,
    minimalRange,
    maximal_range,
    rxPowerRejectionThreshold,
    initiatorsPositions,
    respondersPositions),
  pose_estimator_(respondersPositions, initiatorsPositions)
{
  ranges2d_ = TrilaterationRangeBuffer(respondersPositions.size(), initiatorsPositions.size());
}

//-----------------------------------------------------------------------------
bool R2RRTLSPlugin::compute_leader_pose(ObservationPose & leaderPoseObservation)
{
  if (estimate_leader_pose_()) {
    leaderPoseObservation.first_moment = pose_estimator_.getEstimate();
    leaderPoseObservation.second_moment = pose_estimator_.getEstimateCovariance();
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool R2RRTLSPlugin::estimate_leader_pose_()
{
  return pose_estimator_.init(ranges2d_.data()) &&
         pose_estimator_.estimate(MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, range_std_);
}

//-----------------------------------------------------------------------------
void R2RRTLSPlugin::store_range2d(
  const size_t & initiatorIndex, const size_t & responderIndex, const double & value)
{
  ranges2d_.set(responderIndex, initiatorIndex, value);
}

//-----------------------------------------------------------------------------
void R2RRTLSPlugin::reset_range2d(const size_t & initiatorIndex, const size_t & responderIndex)
{
  ranges2d_.reset(responderIndex, initiatorIndex);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea

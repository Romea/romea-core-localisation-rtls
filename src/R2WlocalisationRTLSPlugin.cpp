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
#include <vector>
#include <string>
#include <optional>

// romea
#include "romea_core_localisation_rtls/R2WLocalisationRTLSPlugin.hpp"

const double MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE = 20;

namespace romea
{

//-----------------------------------------------------------------------------
R2WLocalisationRTLSPlugin::R2WLocalisationRTLSPlugin(
  const double & rangeStd,
  const double & minimalRange,
  const double & maximalRange,
  const uint8_t & rxPowerRejectionThreshold,
  const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
  const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions)
: LocalisationRTLSPlugin(
    rangeStd,
    minimalRange,
    maximalRange,
    rxPowerRejectionThreshold,
    initiatorsPositions,
    respondersPositions),
  reachableResponders_(respondersPositions_, maximalRange),
  poseEstimator_(initiatorsPositions_, respondersPositions)
{
}

//-----------------------------------------------------------------------------
void R2WLocalisationRTLSPlugin::selectRespondersRanges(
  const std::vector<size_t> & respondersIndexes)
{
  // TODO(Jean) refactoring
  for (size_t i = 0; i < respondersPositions_.size(); ++i) {
    auto it = std::find(respondersIndexes.begin(), respondersIndexes.end(), i);
    if (it == respondersIndexes.end()) {
      for (size_t j = 0; j < initiatorsPositions_.size(); ++j) {
        ranges2D_[i][j].reset();
      }
    }
  }
}

//-----------------------------------------------------------------------------
bool R2WLocalisationRTLSPlugin::computePose(ObservationPose & pose_observation)
{
  if (estimatePose_()) {
    pose_observation.firstMoment = poseEstimator_.getEstimate();
    pose_observation.secondMoment = poseEstimator_.getEstimateCovariance();
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool R2WLocalisationRTLSPlugin::estimatePose_()
{
  return poseEstimator_.init(ranges2D_) && poseEstimator_.estimate(
    MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, rangeStd_);
}

}   // namespace romea

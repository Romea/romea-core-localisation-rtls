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
#include "romea_core_localisation_rtls/R2RLocalisationRTLSPlugin.hpp"

const double MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE = 20;

namespace romea
{

//-----------------------------------------------------------------------------
R2RLocalisationRTLSPlugin::R2RLocalisationRTLSPlugin(
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
  poseEstimator_(respondersPositions, initiatorsPositions_)
{
}

//-----------------------------------------------------------------------------
bool R2RLocalisationRTLSPlugin::computeLeaderPose(ObservationPose & leaderPoseObservation)
{
  if (estimateLeaderPose_()) {
    leaderPoseObservation.firstMoment = poseEstimator_.getEstimate();
    leaderPoseObservation.secondMoment = poseEstimator_.getEstimateCovariance();
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool R2RLocalisationRTLSPlugin::estimateLeaderPose_()
{
  return poseEstimator_.init(ranges2D_) && poseEstimator_.estimate(
    MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, rangeStd_);
}

}   // namespace romea

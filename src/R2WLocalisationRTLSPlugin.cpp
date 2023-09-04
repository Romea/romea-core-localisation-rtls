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
  ranges2D_ = TrilaterationRangeBuffer(initiatorsPositions.size(), respondersPositions.size());
}

//-----------------------------------------------------------------------------
void R2WLocalisationRTLSPlugin::selectRespondersRanges(
  const std::vector<size_t> & respondersIndexes)
{
  for (size_t j = 0; j < respondersPositions_.size(); ++j) {
    auto it = std::find(respondersIndexes.begin(), respondersIndexes.end(), j);
    if (it == respondersIndexes.end()) {
      for (size_t i = 0; i < initiatorsPositions_.size(); ++i) {
        resetRange2D(i, j);
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
  // std::cout << " i" << std::endl;
  // for (const auto & i :initiatorsPositions_) {
  //   std::cout << i.transpose() << std::endl;
  // }

  // std::cout << " r" << std::endl;
  // for (const auto & r :respondersPositions_) {
  //   std::cout << r.transpose() << std::endl;
  // }

  // std::cout << ranges2D_ << std::endl;
  return poseEstimator_.init(ranges2D_.data()) && poseEstimator_.estimate(
    MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, rangeStd_);
}

//-----------------------------------------------------------------------------
void R2WLocalisationRTLSPlugin::storeRange2D(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const double & value)
{
  ranges2D_.set(initiatorIndex, responderIndex, value);
}

//-----------------------------------------------------------------------------
void R2WLocalisationRTLSPlugin::resetRange2D(
  const size_t & initiatorIndex,
  const size_t & responderIndex)
{
  ranges2D_.reset(initiatorIndex, responderIndex);
}


}   // namespace romea

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
#include "romea_core_localisation_rtls/R2HLocalisationRTLSPlugin.hpp"


const double MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE = 20;

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
R2HLocalisationRTLSPlugin::R2HLocalisationRTLSPlugin(
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
  positionEstimator_(initiatorsPositions)
{
  assert(respondersPositions.size() == 1);
  ranges2D_ = TrilaterationRangeBuffer(respondersPositions.size(), initiatorsPositions.size());
}

//-----------------------------------------------------------------------------
bool R2HLocalisationRTLSPlugin::computeLeaderPosition(ObservationPosition & leader_position)
{
  if (estimateLeaderPosition_()) {
    leader_position.firstMoment = positionEstimator_.getEstimate();
    leader_position.secondMoment = positionEstimator_.getEstimateCovariance();
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool R2HLocalisationRTLSPlugin::estimateLeaderPosition_()
{
  return positionEstimator_.init(ranges2D_.get(0)) && positionEstimator_.estimate(
    MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, rangeStd_);
}

//-----------------------------------------------------------------------------
void R2HLocalisationRTLSPlugin::storeRange2D(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const double & value)
{
  ranges2D_.set(responderIndex, initiatorIndex, value);
}

//-----------------------------------------------------------------------------
void R2HLocalisationRTLSPlugin::resetRange2D(
  const size_t & initiatorIndex,
  const size_t & responderIndex)
{
  ranges2D_.reset(responderIndex, initiatorIndex);
}

}   // namespace core
}   // namespace romea

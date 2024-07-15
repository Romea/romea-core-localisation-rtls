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
#include <iostream>

// romea
#include "romea_core_localisation_rtls/LocalisationRTLSPlugin.hpp"


namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
LocalisationRTLSPlugin::LocalisationRTLSPlugin(
  const double & rangeStd,
  const double & minimalRange,
  const double & maximalRange,
  const uint8_t & rxPowerRejectionThreshold,
  const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
  const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions)
: rangeStd_(rangeStd),
  ranges2D_(),
  rangingStatus_(minimalRange, maximalRange, rxPowerRejectionThreshold),
  initiatorsPositions_(initiatorsPositions),
  respondersPositions_(respondersPositions)
{
}

//-----------------------------------------------------------------------------
bool LocalisationRTLSPlugin::LocalisationRTLSPlugin::processRangingResult(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const RangingResult & rangingResult,
  ObservationRange & range_observation)
{
  auto status = rangingStatus_.evaluate(rangingResult);
  if (status == RTLSTransceiverRangingStatus::AVAILABLE) {
    range_observation = makeRangeObservation_(initiatorIndex, responderIndex, rangingResult);
    auto range2d = computeRange2D_(initiatorIndex, responderIndex, rangingResult);
    storeRange2D(initiatorIndex, responderIndex, range2d);
    return true;
  } else {
    resetRange2D(initiatorIndex, responderIndex);
    return false;
  }
}

//-----------------------------------------------------------------------------
ObservationRange LocalisationRTLSPlugin::makeRangeObservation_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const RangingResult & rangingResult)
{
  ObservationRange observation;
  observation.firstMoment = rangingResult.range;
  observation.secondMoment = rangeStd_ * rangeStd_;
  observation.initiatorPosition = initiatorsPositions_[initiator_index];
  observation.responderPosition = respondersPositions_[responder_index];
  return observation;
}

//-----------------------------------------------------------------------------
double LocalisationRTLSPlugin::computeRange2D_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const RangingResult & rangingResult)
{
  double dz = initiatorsPositions_[initiator_index].z() -
    respondersPositions_[responder_index].z();

  return std::sqrt(std::pow(rangingResult.range, 2) - dz * dz);
}

}  // namespace core
}  // namespace romea

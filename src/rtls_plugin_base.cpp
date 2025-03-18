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
#include "romea_core_localisation_rtls/rtls_plugin_base.hpp"


namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
RTLSPluginBase::RTLSPluginBase(
  const double & rangeStd,
  const double & minimalRange,
  const double & maximalRange,
  const uint8_t & rxPowerRejectionThreshold,
  const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
  const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions)
: range_std_(rangeStd),
  ranges2d_(),
  ranging_status_(minimalRange, maximalRange, rxPowerRejectionThreshold),
  initiators_positions_(initiatorsPositions),
  responders_positions_(respondersPositions)
{
}

//-----------------------------------------------------------------------------
bool RTLSPluginBase::RTLSPluginBase::process_ranging_result(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const RangingResult & rangingResult,
  ObservationRange & range_observation)
{
  auto status = ranging_status_.evaluate(rangingResult);
  if (status == RTLSTransceiverRangingStatus::AVAILABLE) {
    range_observation = make_range_observation_(initiatorIndex, responderIndex, rangingResult);
    auto range2d = compute_range2d_(initiatorIndex, responderIndex, rangingResult);
    store_range2d(initiatorIndex, responderIndex, range2d);
    return true;
  } else {
    reset_range2d(initiatorIndex, responderIndex);
    return false;
  }
}

//-----------------------------------------------------------------------------
ObservationRange RTLSPluginBase::make_range_observation_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const RangingResult & rangingResult)
{
  ObservationRange observation;
  observation.firstMoment = rangingResult.range;
  observation.secondMoment = range_std_ * range_std_;
  observation.initiator_position = initiators_positions_[initiator_index];
  observation.responder_position = responders_positions_[responder_index];
  return observation;
}

//-----------------------------------------------------------------------------
double RTLSPluginBase::compute_range2d_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const RangingResult & rangingResult)
{
  double dz = initiators_positions_[initiator_index].z() -
    responders_positions_[responder_index].z();

  return std::sqrt(std::pow(rangingResult.range, 2) - dz * dz);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea

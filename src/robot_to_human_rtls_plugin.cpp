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
#include "romea_core_localisation_rtls/robot_to_human_rtls_plugin.hpp"

const double MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE = 20;

namespace romea {
namespace core {
namespace localisation {

//-----------------------------------------------------------------------------
R2HRTLSPlugin::R2HRTLSPlugin(
    const double& rangeStd, const double& minimalRange,
    const double& maximal_range, const uint8_t& rxPowerRejectionThreshold,
    const VectorOfEigenVector<Eigen::Vector3d>& initiatorsPositions,
    const VectorOfEigenVector<Eigen::Vector3d>& respondersPositions)
    : RTLSPluginBase(rangeStd, minimalRange, maximal_range,
                     rxPowerRejectionThreshold, initiatorsPositions,
                     respondersPositions),
      position_estimator_(initiatorsPositions) {
  assert(respondersPositions.size() == 1);
  ranges2d_ = TrilaterationRangeBuffer(respondersPositions.size(),
                                       initiatorsPositions.size());
}

//-----------------------------------------------------------------------------
bool R2HRTLSPlugin::compute_leader_position(
    ObservationPosition& leader_position) {
  if (estimate_leader_position_()) {
    leader_position.first_moment = position_estimator_.getEstimate();
    leader_position.second_moment = position_estimator_.getEstimateCovariance();
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
bool R2HRTLSPlugin::estimate_leader_position_() {
  return position_estimator_.init(ranges2d_.get(0)) &&
         position_estimator_.estimate(
             MAXIMAL_NUMBER_OF_ITERATIONS_TO_ESTIMATE_POSE, range_std_);
}

//-----------------------------------------------------------------------------
void R2HRTLSPlugin::store_range2d(const size_t& initiatorIndex,
                                  const size_t& responderIndex,
                                  const double& value) {
  ranges2d_.set(responderIndex, initiatorIndex, value);
}

//-----------------------------------------------------------------------------
void R2HRTLSPlugin::reset_range2d(const size_t& initiatorIndex,
                                  const size_t& responderIndex) {
  ranges2d_.reset(responderIndex, initiatorIndex);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea

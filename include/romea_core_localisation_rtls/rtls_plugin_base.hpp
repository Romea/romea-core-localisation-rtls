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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__RTLS_PLUGIN_BASE_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__RTLS_PLUGIN_BASE_HPP_

// std
#include <optional>
#include <string>
#include <vector>

// romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_localisation/observation_range.hpp"
#include "romea_core_localisation_rtls/trilateration_data_buffer.hpp"
#include "romea_core_rtls/ranging/result.hpp"
#include "romea_core_rtls/ranging/status.hpp"

namespace romea {
namespace core {
namespace localisation {

class RTLSPluginBase {
 public:
  using RangingResult = RTLSRangingResult;
  using RangingStatus = RTLSRangingStatus;
  using RangingStatusEvaluator = RTLSRangingStatusEvaluator;

  using Range = std::optional<double>;
  using RangeVector = std::vector<Range>;
  using RangeArray = std::vector<RangeVector>;

 public:
  RTLSPluginBase(
      const double& rangeStd, const double& minimalRange,
      const double& maximal_range, const uint8_t& rxPowerRejectionThreshold,
      const VectorOfEigenVector<Eigen::Vector3d>& initiatorsPositions,
      const VectorOfEigenVector<Eigen::Vector3d>& respondersPositions);

  virtual ~RTLSPluginBase() = default;

  bool process_ranging_result(const size_t& initiatorIndex,
                              const size_t& responderIndex,
                              const RangingResult& rangingResult,
                              ObservationRange& observation);

 protected:
  virtual void store_range2d(const size_t& initiatorIndex,
                             const size_t& responderIndex,
                             const double& value) = 0;

  virtual void reset_range2d(const size_t& initiatorIndex,
                             const size_t& responderIndex) = 0;

  virtual double compute_range2d_(const size_t& initiatorIndex,
                                  const size_t& responderIndex,
                                  const RangingResult& rangingResult);

  ObservationRange make_range_observation_(const size_t& initiatorIndex,
                                           const size_t& responderIndex,
                                           const RangingResult& rangingResult);

 protected:
  double range_std_;
  TrilaterationRangeBuffer ranges2d_;
  RangingStatusEvaluator ranging_status_;

  VectorOfEigenVector3d initiators_positions_;
  VectorOfEigenVector3d responders_positions_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__RTLS_PLUGIN_BASE_HPP_

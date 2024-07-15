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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__LOCALISATIONRTLSPLUGIN_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__LOCALISATIONRTLSPLUGIN_HPP_

// std
#include <optional>
#include <vector>
#include <string>

// romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_rtls_transceiver/RTLSTransceiverRangingResult.hpp"
#include "romea_core_rtls_transceiver/RTLSTransceiverRangingStatus.hpp"
#include "romea_core_localisation_rtls/TrilaterationDataBuffer.hpp"
#include "romea_core_localisation/ObservationRange.hpp"

namespace romea
{
namespace core
{

class LocalisationRTLSPlugin
{
public:
  using RangingResult = RTLSTransceiverRangingResult;
  using RangingStatus = RTLSTransceiverRangingStatus;
  using RangingStatusEvaluator = RTLSTransceiverRangingStatusEvaluator;

  using Range = std::optional<double>;
  using RangeVector = std::vector<Range>;
  using RangeArray = std::vector<RangeVector>;

public:
  LocalisationRTLSPlugin(
    const double & rangeStd,
    const double & minimalRange,
    const double & maximalRange,
    const uint8_t & rxPowerRejectionThreshold,
    const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
    const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions);

  virtual ~LocalisationRTLSPlugin() = default;

  bool processRangingResult(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const RangingResult & rangingResult,
    ObservationRange & observation);

protected:
  virtual void storeRange2D(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const double & value) = 0;

  virtual void resetRange2D(
    const size_t & initiatorIndex,
    const size_t & responderIndex) = 0;

  virtual double computeRange2D_(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const RangingResult & rangingResult);

  ObservationRange makeRangeObservation_(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const RangingResult & rangingResult);

protected:
  double rangeStd_;
  TrilaterationRangeBuffer ranges2D_;
  RangingStatusEvaluator rangingStatus_;

  VectorOfEigenVector3d initiatorsPositions_;
  VectorOfEigenVector3d respondersPositions_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__LOCALISATIONRTLSPLUGIN_HPP_

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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__R2RLOCALISATIONRTLSPLUGIN_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__R2RLOCALISATIONRTLSPLUGIN_HPP_

// std
#include <optional>
#include <vector>
#include <string>

// romea
#include "romea_core_rtls/trilateration/RTLSPose2DEstimator.hpp"
#include "romea_core_localisation_rtls/LocalisationRTLSPlugin.hpp"
#include "romea_core_localisation/ObservationPose.hpp"

namespace romea
{
namespace core
{

class R2RLocalisationRTLSPlugin : public LocalisationRTLSPlugin
{
public:
  R2RLocalisationRTLSPlugin(
    const double & rangeStd,
    const double & minimalRange,
    const double & maximalRange,
    const uint8_t & rxPowerRejectionThreshold,
    const VectorOfEigenVector<Eigen::Vector3d> & initiatorsPositions,
    const VectorOfEigenVector<Eigen::Vector3d> & respondersPositions);

  virtual ~R2RLocalisationRTLSPlugin() = default;

  bool computeLeaderPose(ObservationPose & leaserPose);

private:
  void storeRange2D(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const double & value) override;

  void resetRange2D(
    const size_t & initiatorIndex,
    const size_t & responderIndex) override;

  bool estimateLeaderPose_();

private:
  RTLSPose2DEstimator poseEstimator_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__R2RLOCALISATIONRTLSPLUGIN_HPP_

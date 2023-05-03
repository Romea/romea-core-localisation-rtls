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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_POSE_ESTIMATOR_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_POSE_ESTIMATOR_HPP_

// std
#include <vector>

// romea
#include "romea_core_common/regression/leastsquares/NLSE.hpp"
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_ranges.hpp"

namespace romea
{

class RTLSLocalisationPoseEstimator : public NLSE<double>
{
public:
  RTLSLocalisationPoseEstimator();

  explicit RTLSLocalisationPoseEstimator(const double & estimateEpsilon);

  virtual ~RTLSLocalisationPoseEstimator() = default;

  void loadGeometry(
    const VectorOfEigenVector3d & targetTagPositions,
    const VectorOfEigenVector3d & referenceTagPositions);

  bool initR2R(const RTLSLocalisationRangeArray & ranges);

  bool initR2W(
    const RTLSLocalisationRangeArray & ranges,
    const std::vector<size_t> & referenceTagIndexes);

private:
  void computeGuess_()override;

  void computeJacobianAndY_()override;

private:
  VectorOfEigenVector2d targetTagPositions_;
  VectorOfEigenVector2d referenceTagPositions_;
  std::vector<std::vector<double>> ranges_;
  std::vector<std::vector<size_t>> indexesOfAvailableRanges_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_POSE_ESTIMATOR_HPP_

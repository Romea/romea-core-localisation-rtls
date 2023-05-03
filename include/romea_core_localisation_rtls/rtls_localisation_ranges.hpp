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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_RANGES_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_RANGES_HPP_

// Eigen
#include <Eigen/Core>

// std
#include <optional>
#include <fstream>
#include <vector>

// romea
#include "romea_core_rtls/RTLSRange.hpp"
#include "romea_core_common/pointset/PointSet.hpp"

namespace romea
{


// TODO(jean) change _name
class RTLSLocalisationRange
{
public:
  RTLSLocalisationRange(
    const int & initiatorId,
    const Eigen::Vector3d & initiatorPosition,
    const int & responderId,
    const Eigen::Vector3d & responderPosition,
    const double & bias);

  bool isAvailable()const;

  const RTLSRange & get()const;

  void set(const RTLSRange & range);

  void reset();


  double computeUnbiasedRange3D()const;

  double computeUnbiasedRange2D()const;

  void log()const;

private:
  std::optional<RTLSRange> range_;

  double bias_;
  double zOffset_;
  double squareZOffset_;

  mutable std::ofstream logFile_;
};


using RTLSLocalisationRangeVector = std::vector<RTLSLocalisationRange>;
using RTLSLocalisationRangeArray = std::vector<RTLSLocalisationRangeVector>;

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_RANGES_HPP_

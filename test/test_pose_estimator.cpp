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

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_pose_estimator.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_simple_trilateration.hpp"

//-----------------------------------------------------------------------------
TEST(TestRtlsPoseEstimator, testRtlsPoseEstimator)
{
  std::vector<int> tagIds{0, 1, 2};
  std::vector<int> anchorIds{0, 1, 2};

  romea::VectorOfEigenVector3d tagPositions;
  tagPositions.emplace_back(0, -0.3, 1.01);
  tagPositions.emplace_back(0, 0.3, 1.01);
  tagPositions.emplace_back(0.44, 0, 0.71);

  romea::VectorOfEigenVector3d anchorPositions;
  anchorPositions.emplace_back(0, -0.21, 0.39);
  anchorPositions.emplace_back(0, 0.21, 0.39);
  anchorPositions.emplace_back(0.85, 0, 0.44);

  romea::RTLSLocalisationRangeArray ranges(3);

  ranges.reserve(tagIds.size());
  for (size_t i = 0; i < anchorIds.size(); ++i) {
    ranges[i].reserve(tagIds.size());
    for (size_t j = 0; j < tagIds.size(); ++j) {
      ranges[i].emplace_back(
        tagIds[j],
        tagPositions[j],
        anchorIds[i],
        anchorPositions[i],
        0.0);
    }
  }

  romea::RTLSLocalisationPoseEstimator estimator(0.001);
  estimator.loadGeometry(anchorPositions, tagPositions);

  double rho = 5;
  double theta = 0;
  double course = -M_PI;
  for (; theta < 2 * M_PI; theta += M_PI / 4, course += M_PI / 3) {
    Eigen::Matrix3d R = romea::eulerAnglesToRotation3D(
      Eigen::Vector3d(0, 0, romea::between0And2Pi(course)));
    Eigen::Vector3d T(rho * std::cos(theta), rho * std::cos(theta), 0);

    double t = 10.000;
    for (size_t i = 0; i < anchorIds.size(); i++) {
      for (size_t j = 0; j < tagIds.size(); j++) {
        ranges[i][j].set(
          {romea::durationFromSecond(t),
            ((R * anchorPositions[i] + T) - tagPositions[j]).norm(), 0, 0});

        t += 0.050;
      }
    }


    EXPECT_TRUE(estimator.initR2R(ranges));
    EXPECT_TRUE(estimator.estimate(10, 0.02));
    EXPECT_NEAR(T[0], estimator.getEstimate()[0], 0.01);
    EXPECT_NEAR(T[1], estimator.getEstimate()[1], 0.01);
    EXPECT_NEAR(romea::betweenMinusPiAndPi(course - estimator.getEstimate()[2]), 0.0, 0.01);
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

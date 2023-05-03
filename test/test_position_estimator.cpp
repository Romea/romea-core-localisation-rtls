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

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_position_estimator.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_simple_trilateration.hpp"


//-----------------------------------------------------------------------------
TEST(TestRtlsPositionEstimator, testPositionEstimatorWithTwoAnchors)
{
  auto tag0Position = Eigen::Vector3d(5.0, 2.0, 2.0);
  auto anchor0Position = Eigen::Vector3d(0.0, 0.3, 1);
  auto anchor1Position = Eigen::Vector3d(0.0, -0.3, 1);
  double r00 = (tag0Position - anchor0Position).norm();
  double r01 = (tag0Position - anchor1Position).norm();

  romea::RTLSLocalisationRangeVector ranges;
  ranges.emplace_back(0, tag0Position, 0, anchor0Position, 0.0);
  ranges.emplace_back(0, tag0Position, 1, anchor1Position, 0.0);
  ranges[0].set({romea::durationFromSecond(10.000), r00, 0, 0});
  ranges[1].set({romea::durationFromSecond(10.050), r01, 0, 0});

  romea::RTLSLocalisationPositionEstimator estimator(0.001);
  estimator.loadGeometry({anchor0Position, anchor1Position});
  EXPECT_TRUE(estimator.init(ranges));
  EXPECT_TRUE(estimator.estimate(20, 0.02));

  auto tag0EstimatedPosition = estimator.getEstimate();

  EXPECT_NEAR(tag0Position.x(), tag0EstimatedPosition.x(), 0.001);
  EXPECT_NEAR(tag0Position.y(), tag0EstimatedPosition.y(), 0.001);
}

//-----------------------------------------------------------------------------
TEST(TestRtlsPositionEstimator, testPositionEstimatorWithThreeAnchorsUp)
{
  auto tag0Position = Eigen::Vector3d(-4, 6, 1);
  auto anchor0Position = Eigen::Vector3d(0, 0.6, 2);
  auto anchor1Position = Eigen::Vector3d(0, -0.6, 1.5);
  auto anchor2Position = Eigen::Vector3d(1, 0, 1.8);
  double r00 = (tag0Position - anchor0Position).norm();
  double r01 = (tag0Position - anchor1Position).norm();
  double r02 = (tag0Position - anchor2Position).norm();

  romea::RTLSLocalisationRangeVector ranges;
  ranges.emplace_back(0, tag0Position, 0, anchor0Position, 0.0);
  ranges.emplace_back(0, tag0Position, 1, anchor1Position, 0.0);
  ranges.emplace_back(0, tag0Position, 2, anchor2Position, 0.0);
  ranges[0].set({romea::durationFromSecond(10.000), r00, 0, 0});
  ranges[1].set({romea::durationFromSecond(10.050), r01, 0, 0});
  ranges[2].set({romea::durationFromSecond(10.100), r02, 0, 0});

  romea::RTLSLocalisationPositionEstimator estimator(0.001);
  estimator.loadGeometry({anchor0Position, anchor1Position, anchor2Position});
  EXPECT_TRUE(estimator.init(ranges));
  EXPECT_TRUE(estimator.estimate(20, 0.02));

  auto tag0EstimatedPosition = estimator.getEstimate();
  EXPECT_NEAR(tag0Position.x(), tag0EstimatedPosition.x(), 0.001);
  EXPECT_NEAR(tag0Position.y(), tag0EstimatedPosition.y(), 0.001);
}

//-----------------------------------------------------------------------------
TEST(TestRtlsPositionEstimator, testPositionEstimatorWithThreeAnchorsDown)
{
  auto tag0Position = Eigen::Vector3d(-6, -7, 1);
  auto anchor0Position = Eigen::Vector3d(0, 0.6, 2);
  auto anchor1Position = Eigen::Vector3d(0, -0.6, 1.5);
  auto anchor2Position = Eigen::Vector3d(-2, 0, 1.8);
  double r00 = (tag0Position - anchor0Position).norm();
  double r01 = (tag0Position - anchor1Position).norm();
  double r02 = (tag0Position - anchor2Position).norm();

  romea::RTLSLocalisationRangeVector ranges;
  ranges.emplace_back(0, tag0Position, 0, anchor0Position, 0.0);
  ranges.emplace_back(0, tag0Position, 1, anchor1Position, 0.0);
  ranges.emplace_back(0, tag0Position, 2, anchor2Position, 0.0);
  ranges[0].set({romea::durationFromSecond(10.000), r00, 0, 0});
  ranges[1].set({romea::durationFromSecond(10.050), r01, 0, 0});
  ranges[2].set({romea::durationFromSecond(10.100), r02, 0, 0});


  romea::RTLSLocalisationPositionEstimator estimator(0.001);
  estimator.loadGeometry({anchor0Position, anchor1Position, anchor2Position});
  EXPECT_TRUE(estimator.init(ranges));
  EXPECT_TRUE(estimator.estimate(20, 0.02));

  auto tag0EstimatedPosition = estimator.getEstimate();
  EXPECT_NEAR(tag0Position.x(), tag0EstimatedPosition.x(), 0.001);
  EXPECT_NEAR(tag0Position.y(), tag0EstimatedPosition.y(), 0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include <cmath>

#include "gtest/gtest.h"

#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/time/Time.hpp"
#include "romea_core_localisation_rtls/robot_to_world_rtls_plugin.hpp"

namespace
{

using romea::core::RTLSRangingResult;
using romea::core::VectorOfEigenVector3d;
using romea::core::betweenMinusPiAndPi;
using romea::core::durationFromSecond;
using romea::core::eulerAnglesToRotation3D;
using romea::core::localisation::ObservationPose;
using romea::core::localisation::ObservationRange;
using romea::core::localisation::R2WRTLSPlugin;

RTLSRangingResult makeRangingResult(const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  return {durationFromSecond(1.0), (from - to).norm(), 20, 21};
}

TEST(TestRobotToWorldRTLSPlugin, estimatesPoseFromAvailableRanges)
{
  VectorOfEigenVector3d initiators_positions;
  initiators_positions.emplace_back(0.0, -0.3, 1.01);
  initiators_positions.emplace_back(0.0, 0.3, 1.01);
  initiators_positions.emplace_back(0.44, 0.0, 0.71);

  VectorOfEigenVector3d responders_positions;
  responders_positions.emplace_back(0.0, -0.21, 0.39);
  responders_positions.emplace_back(0.0, 0.21, 0.39);
  responders_positions.emplace_back(0.85, 0.0, 0.44);

  const Eigen::Vector3d expected_translation(4.0, 3.0, 0.0);
  const double expected_yaw = -0.5;
  const Eigen::Matrix3d rotation =
    eulerAnglesToRotation3D(Eigen::Vector3d(0.0, 0.0, expected_yaw));

  R2WRTLSPlugin plugin(0.001, 0.1, 20.0, 10, initiators_positions, responders_positions);

  ObservationRange range_observation;
  for (size_t initiator_index = 0; initiator_index < initiators_positions.size();
    ++initiator_index)
  {
    for (size_t responder_index = 0; responder_index < responders_positions.size();
      ++responder_index)
    {
      const auto initiator_position =
        rotation * initiators_positions[initiator_index] + expected_translation;
      EXPECT_TRUE(plugin.process_ranging_result(
        initiator_index,
        responder_index,
        makeRangingResult(initiator_position, responders_positions[responder_index]),
        range_observation));
    }
  }

  ObservationPose pose;
  ASSERT_TRUE(plugin.compute_pose(pose));

  EXPECT_NEAR(pose.first_moment.x(), expected_translation.x(), 0.01);
  EXPECT_NEAR(pose.first_moment.y(), expected_translation.y(), 0.01);
  EXPECT_NEAR(betweenMinusPiAndPi(pose.first_moment.z() - expected_yaw), 0.0, 0.01);
}

TEST(TestRobotToWorldRTLSPlugin, failsAfterUnselectedResponderRangesAreReset)
{
  VectorOfEigenVector3d initiators_positions;
  initiators_positions.emplace_back(0.0, -0.3, 1.01);
  initiators_positions.emplace_back(0.0, 0.3, 1.01);
  initiators_positions.emplace_back(0.44, 0.0, 0.71);

  VectorOfEigenVector3d responders_positions;
  responders_positions.emplace_back(0.0, -0.21, 0.39);
  responders_positions.emplace_back(0.0, 0.21, 0.39);
  responders_positions.emplace_back(0.85, 0.0, 0.44);

  R2WRTLSPlugin plugin(0.001, 0.1, 20.0, 10, initiators_positions, responders_positions);

  ObservationRange range_observation;
  for (size_t initiator_index = 0; initiator_index < initiators_positions.size();
    ++initiator_index)
  {
    for (size_t responder_index = 0; responder_index < responders_positions.size();
      ++responder_index)
    {
      EXPECT_TRUE(plugin.process_ranging_result(
        initiator_index,
        responder_index,
        makeRangingResult(initiators_positions[initiator_index], responders_positions[responder_index]),
        range_observation));
    }
  }

  plugin.select_responders_ranges({0, 1});

  ObservationPose pose;
  EXPECT_FALSE(plugin.compute_pose(pose));
}

}  // namespace

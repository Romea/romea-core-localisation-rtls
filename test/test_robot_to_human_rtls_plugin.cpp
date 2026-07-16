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

#include "gtest/gtest.h"

#include "romea_core_common/time/Time.hpp"
#include "romea_core_localisation_rtls/robot_to_human_rtls_plugin.hpp"

namespace
{

using romea::core::RTLSRangingResult;
using romea::core::VectorOfEigenVector3d;
using romea::core::durationFromSecond;
using romea::core::localisation::ObservationPosition;
using romea::core::localisation::ObservationRange;
using romea::core::localisation::R2HRTLSPlugin;

RTLSRangingResult makeRangingResult(const Eigen::Vector3d & initiator, const Eigen::Vector3d & responder)
{
  return {durationFromSecond(1.0), (initiator - responder).norm(), 20, 21};
}

TEST(TestRobotToHumanRTLSPlugin, estimatesLeaderPositionFromAvailableRanges)
{
  VectorOfEigenVector3d initiators_positions;
  initiators_positions.emplace_back(0.0, 0.3, 1.0);
  initiators_positions.emplace_back(0.0, -0.3, 1.0);
  initiators_positions.emplace_back(1.0, 0.0, 1.0);

  VectorOfEigenVector3d responders_positions;
  responders_positions.emplace_back(5.0, 2.0, 1.0);

  R2HRTLSPlugin plugin(0.001, 0.1, 20.0, 10, initiators_positions, responders_positions);

  ObservationRange range_observation;
  for (size_t i = 0; i < initiators_positions.size(); ++i) {
    EXPECT_TRUE(plugin.process_ranging_result(
      i, 0, makeRangingResult(initiators_positions[i], responders_positions[0]), range_observation));
  }

  ObservationPosition leader_position;
  ASSERT_TRUE(plugin.compute_leader_position(leader_position));

  EXPECT_NEAR(leader_position.first_moment.x(), responders_positions[0].x(), 0.01);
  EXPECT_NEAR(leader_position.first_moment.y(), responders_positions[0].y(), 0.01);
}

TEST(TestRobotToHumanRTLSPlugin, failsToEstimateAfterRangeReset)
{
  VectorOfEigenVector3d initiators_positions;
  initiators_positions.emplace_back(0.0, 0.3, 1.0);
  initiators_positions.emplace_back(0.0, -0.3, 1.0);

  VectorOfEigenVector3d responders_positions;
  responders_positions.emplace_back(5.0, 2.0, 1.0);

  R2HRTLSPlugin plugin(0.001, 0.1, 20.0, 10, initiators_positions, responders_positions);

  ObservationRange range_observation;
  for (size_t i = 0; i < initiators_positions.size(); ++i) {
    EXPECT_TRUE(plugin.process_ranging_result(
      i, 0, makeRangingResult(initiators_positions[i], responders_positions[0]), range_observation));
  }

  const RTLSRangingResult unavailable_result{durationFromSecond(2.0), 30.0, 20, 21};
  EXPECT_FALSE(plugin.process_ranging_result(0, 0, unavailable_result, range_observation));

  ObservationPosition leader_position;
  EXPECT_FALSE(plugin.compute_leader_position(leader_position));
}

}  // namespace

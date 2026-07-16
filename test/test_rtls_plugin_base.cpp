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
#include "romea_core_localisation_rtls/rtls_plugin_base.hpp"

namespace
{

using romea::core::RTLSRangingResult;
using romea::core::VectorOfEigenVector3d;
using romea::core::durationFromSecond;
using romea::core::localisation::ObservationRange;
using romea::core::localisation::RTLSPluginBase;

class TestRTLSPlugin : public RTLSPluginBase
{
public:
  TestRTLSPlugin(
    const VectorOfEigenVector3d & initiators_positions,
    const VectorOfEigenVector3d & responders_positions)
  : RTLSPluginBase(0.2, 0.1, 20.0, 10, initiators_positions, responders_positions),
    stored_range2d(),
    reset_called(false)
  {
  }

  std::optional<double> stored_range2d;
  bool reset_called;

private:
  void store_range2d(
    const size_t & initiator_index, const size_t & responder_index, const double & value) override
  {
    EXPECT_EQ(initiator_index, 0u);
    EXPECT_EQ(responder_index, 0u);
    stored_range2d = value;
  }

  void reset_range2d(const size_t & initiator_index, const size_t & responder_index) override
  {
    EXPECT_EQ(initiator_index, 0u);
    EXPECT_EQ(responder_index, 0u);
    stored_range2d.reset();
    reset_called = true;
  }
};

TEST(TestRTLSPluginBase, createsRangeObservationAndStoresHorizontalRange)
{
  VectorOfEigenVector3d initiators_positions;
  initiators_positions.emplace_back(1.0, 2.0, 3.0);

  VectorOfEigenVector3d responders_positions;
  responders_positions.emplace_back(4.0, 6.0, 0.0);

  TestRTLSPlugin plugin(initiators_positions, responders_positions);

  ObservationRange observation;
  const RTLSRangingResult result{durationFromSecond(1.0), 5.0, 20, 21};

  EXPECT_TRUE(plugin.process_ranging_result(0, 0, result, observation));

  EXPECT_DOUBLE_EQ(observation.first_moment, 5.0);
  EXPECT_DOUBLE_EQ(observation.second_moment, 0.04);
  EXPECT_TRUE(observation.initiator_position.isApprox(initiators_positions[0]));
  EXPECT_TRUE(observation.responder_position.isApprox(responders_positions[0]));

  ASSERT_TRUE(plugin.stored_range2d.has_value());
  EXPECT_DOUBLE_EQ(plugin.stored_range2d.value(), 4.0);
  EXPECT_FALSE(plugin.reset_called);
}

TEST(TestRTLSPluginBase, resetsHorizontalRangeWhenRangingIsUnavailable)
{
  VectorOfEigenVector3d initiators_positions;
  initiators_positions.emplace_back(0.0, 0.0, 0.0);

  VectorOfEigenVector3d responders_positions;
  responders_positions.emplace_back(1.0, 0.0, 0.0);

  TestRTLSPlugin plugin(initiators_positions, responders_positions);
  plugin.stored_range2d = 1.0;

  ObservationRange observation;
  const RTLSRangingResult result{durationFromSecond(1.0), 30.0, 20, 21};

  EXPECT_FALSE(plugin.process_ranging_result(0, 0, result, observation));
  EXPECT_FALSE(plugin.stored_range2d.has_value());
  EXPECT_TRUE(plugin.reset_called);
}

}  // namespace

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

#include "romea_core_localisation_rtls/trilateration_data_buffer.hpp"

namespace
{

using romea::core::localisation::TrilaterationRangeBuffer;

TEST(TestTrilaterationRangeBuffer, storesAndResetsIndividualRanges)
{
  TrilaterationRangeBuffer buffer(2, 3);

  EXPECT_EQ(buffer.data().size(), 2u);
  EXPECT_EQ(buffer.get(0).size(), 3u);
  EXPECT_FALSE(buffer.get(1, 2).has_value());

  buffer.set(1, 2, 4.5);

  ASSERT_TRUE(buffer.get(1, 2).has_value());
  EXPECT_DOUBLE_EQ(buffer.get(1, 2).value(), 4.5);

  buffer.reset(1, 2);

  EXPECT_FALSE(buffer.get(1, 2).has_value());
}

TEST(TestTrilaterationRangeBuffer, resetsRowsAndWholeBuffer)
{
  TrilaterationRangeBuffer buffer(2, 2);
  buffer.set(0, 0, 1.0);
  buffer.set(0, 1, 2.0);
  buffer.set(1, 0, 3.0);

  buffer.reset(0);

  EXPECT_FALSE(buffer.get(0, 0).has_value());
  EXPECT_FALSE(buffer.get(0, 1).has_value());
  EXPECT_TRUE(buffer.get(1, 0).has_value());

  buffer.reset();

  EXPECT_FALSE(buffer.get(1, 0).has_value());
}

}  // namespace

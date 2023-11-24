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
#include <ostream>
#include <optional>

// romea
#include "romea_core_localisation_rtls/TrilaterationDataBuffer.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
TrilaterationRangeBuffer::TrilaterationRangeBuffer()
: ranges_()
{

}

//-----------------------------------------------------------------------------
TrilaterationRangeBuffer::TrilaterationRangeBuffer(
  const size_t rows,
  const size_t cols)
: ranges_(rows, RangeVector(cols, Range()))
{

}

//-----------------------------------------------------------------------------
void TrilaterationRangeBuffer::TrilaterationRangeBuffer::set(
  const size_t & row,
  const size_t & col,
  const double & value)
{
  ranges_[row][col] = value;
}

//-----------------------------------------------------------------------------
const TrilaterationRangeBuffer::Range & TrilaterationRangeBuffer::get(
  const size_t & row,
  const size_t & col) const
{
  return ranges_[row][col];
}

//-----------------------------------------------------------------------------
const TrilaterationRangeBuffer::RangeVector & TrilaterationRangeBuffer::get(const size_t & row)
const
{
  return ranges_[row];
}

//-----------------------------------------------------------------------------
void TrilaterationRangeBuffer::reset(
  const size_t & row,
  const size_t & col)
{
  ranges_[row][col].reset();
}

//-----------------------------------------------------------------------------
void TrilaterationRangeBuffer::reset(const size_t & row)
{
  for (auto & range : ranges_[row]) {
    range.reset();
  }
}

//-----------------------------------------------------------------------------
void TrilaterationRangeBuffer::reset()
{
  for (size_t row = 0; row < ranges_.size(); ++row) {
    reset(row);
  }
}

//-----------------------------------------------------------------------------
const TrilaterationRangeBuffer::RangeArray & TrilaterationRangeBuffer::data() const
{
  return ranges_;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const TrilaterationRangeBuffer & buffer)
{
  for (const auto & rangeVector : buffer.data()) {
    for (const auto & range : rangeVector) {
      if (range.has_value()) {
        os << range.value();
      } else {
        os << "nan";
      }
      os << " ";
    }
    os << std::endl;
  }
  os << std::endl;
  return os;
}

}  // namespace core
}  // namespace romea

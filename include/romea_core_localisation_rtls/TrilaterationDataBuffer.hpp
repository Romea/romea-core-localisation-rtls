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

#ifndef ROMEA_CORE_LOCALISATION_RTLS__TRILATERATIONDATABUFFER_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__TRILATERATIONDATABUFFER_HPP_

// std
#include <optional>
#include <vector>
#include <iostream>

namespace romea
{
namespace core
{


class TrilaterationRangeBuffer
{
public:
  using Range = std::optional<double>;
  using RangeVector = std::vector<Range>;
  using RangeArray = std::vector<RangeVector>;

  TrilaterationRangeBuffer();

  TrilaterationRangeBuffer(
    const size_t rows,
    const size_t cols);

  void resize(
    const size_t rows,
    const size_t cols);

  void set(
    const size_t & row,
    const size_t & col,
    const double & value);

  const Range & get(
    const size_t & row,
    const size_t & col)const;

  const RangeVector & get(const size_t & row)const;

  void reset(
    const size_t & row,
    const size_t & col);

  void reset(const size_t & row);

  void reset();

  const RangeArray & data()const;

private:
  RangeArray ranges_;
};

std::ostream & operator<<(std::ostream & os, const TrilaterationRangeBuffer & buffer);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__TRILATERATIONDATABUFFER_HPP_

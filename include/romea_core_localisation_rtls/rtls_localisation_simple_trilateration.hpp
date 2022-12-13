#ifndef ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_SIMPLE_TRILATERATION_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_SIMPLE_TRILATERATION_HPP_

// std
#include <vector>

// romea
#include <romea_core_common/containers/Eigen/VectorOfEigenVector.hpp>

namespace romea {


class SimpleTrilateration2D
{

public :

  static Eigen::Vector2d compute(const VectorOfEigenVector2d & tagPositions,
                                 const std::vector<double> & ranges);

  static Eigen::Vector2d compute(const VectorOfEigenVector2d & tagPositions,
                                 const std::vector<double> & ranges,
                                 const std::vector<size_t> & rangesIndexes);

private :

  static Eigen::Vector2d compute_(const VectorOfEigenVector2d & tagPositions,
                                  const std::vector<double> & ranges,
                                  const size_t & i,
                                  const size_t & j);

  static Eigen::Vector2d compute_(const VectorOfEigenVector2d & tagPositions,
                                  const std::vector<double> & ranges,
                                  const std::vector<size_t> & rangesIndexes,
                                  const size_t & i,
                                  const size_t & j);

  static VectorOfEigenVector2d compute_(const Eigen::Vector2d & p1,
                                        const Eigen::Vector2d & p2,
                                        const double & r1,
                                        const double & r2);
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_SIMPLE_TRILATERATION_HPP_

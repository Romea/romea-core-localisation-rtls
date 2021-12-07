#ifndef __RTLSLocalisationSimpleTrilateration_HPP__
#define __RTLSLocalisationSimpleTrilateration_HPP__

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

}
#endif

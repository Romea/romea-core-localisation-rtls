// romea
#include "romea_core_localisation_rtls/rtls_localisation_simple_trilateration.hpp"

// std
#include <algorithm>
// #include <iostream>

namespace romea
{

//-----------------------------------------------------------------------------
VectorOfEigenVector2d SimpleTrilateration2D::compute_(const Eigen::Vector2d & p1 ,
                                                      const Eigen::Vector2d & p2,
                                                      const double & r1,
                                                      const double & r2)

{
  double dx = p2.x()-p1.x();
  double dy = p2.y()-p1.y();

  double base = std::sqrt(dx*dx+dy*dy);
  double theta = std::atan2(dy, dx);
  //  std::cout << (base*base+r1*r1-r2*r2)/(2*base*r1) << std::endl;
  double alpha = std::acos(std::max(std::min((base*base+r1*r1-r2*r2)/(2*base*r1), 1.), -1.));

  VectorOfEigenVector2d solutions(2, p1);
  solutions[0].x()+=r1*std::cos(theta+alpha);
  solutions[0].y()+=r1*std::sin(theta+alpha);
  solutions[1].x()+=r1*std::cos(theta-alpha);
  solutions[1].y()+=r1*std::sin(theta-alpha);

  if (solutions[0].x() < 0)
  {
    std::swap(solutions[0], solutions[1]);
  }

  //  std::cout << " p1 "<<p1.transpose()<< std::endl;
  //  std::cout << " p2 "<<p2.transpose()<< std::endl;
  //  std::cout << " r1 "<<r1<< std::endl;
  //  std::cout << " r2 "<<r2<< std::endl;

  //  std::cout << " base "<<base<< std::endl;
  //  std::cout << " theta "<<theta*180/M_PI<< std::endl;
  //  std::cout << " alpha "<<alpha*180/M_PI<< std::endl;

  //  std::cout << " solutions[0] "<<solutions[0].transpose()<< std::endl;
  //  std::cout << " solutions[1] "<<solutions[1].transpose()<< std::endl;

  return solutions;
}


//-----------------------------------------------------------------------------
Eigen::Vector2d SimpleTrilateration2D::compute_(const VectorOfEigenVector2d & tagPositions,
                                                const std::vector<double> & ranges,
                                                const size_t & i,
                                                const size_t & j)

{
  std::vector<double> errors(2, 0);

  VectorOfEigenVector2d solutions = compute_(tagPositions[i],
                                             tagPositions[j],
                                             ranges[i],
                                             ranges[j]);
  size_t k = (j+1)%ranges.size();
  for (;k != i; k = (k+1)%ranges.size())
  {
    errors[0]+=std::abs((tagPositions[k]-solutions[0]).norm()-ranges[k]);
    errors[1]+=std::abs((tagPositions[k]-solutions[1]).norm()-ranges[k]);
  }

//  std::cout <<" errors "<< errors[0] <<" "<<errors[1]<< std::endl;

  return errors[0] <= errors[1] ? solutions[0] : solutions[1];
}


//-----------------------------------------------------------------------------
Eigen::Vector2d SimpleTrilateration2D::compute_(const VectorOfEigenVector2d & tagPositions,
                                                const std::vector<double> & ranges,
                                                const std::vector<size_t> & rangesIndexes,
                                                const size_t & i,
                                                const size_t & j)

{
  std::vector<double> errors(2, 0);

  VectorOfEigenVector2d solutions = compute_(tagPositions[i],
                                             tagPositions[j],
                                             ranges[rangesIndexes[i]],
                                             ranges[rangesIndexes[j]]);

  size_t k = (j+1)%ranges.size();
  for (;k != i;k = (k+1)%ranges.size())
  {
    errors[0]+=std::abs((tagPositions[k]-solutions[0]).norm()-ranges[rangesIndexes[k]]);
    errors[1]+=std::abs((tagPositions[k]-solutions[1]).norm()-ranges[rangesIndexes[k]]);
  }

  //  std::cout <<" errors "<< errors[0] <<" "<<errors[1]<< std::endl;
  return errors[0] <= errors[1] ? solutions[0] : solutions[1];
}

//-----------------------------------------------------------------------------
Eigen::Vector2d SimpleTrilateration2D::compute(const VectorOfEigenVector2d & tagPositions,
                                               const std::vector<double> & ranges)
{
  assert(tagPositions.size() >= 2);
  assert(tagPositions.size() == ranges.size());

  if (tagPositions.size() == 2)
  {
    return compute_(tagPositions, ranges, 0, 1);
  } else {
    Eigen::Vector2d solution = Eigen::Vector2d::Zero();
    for (size_t i=0, j = 1; i < ranges.size(); ++i, j=(j+1)%ranges.size())
    {
      Eigen::Vector2d solution_ = compute_(tagPositions, ranges, i, j);
      solution+=solution_;
    }
    return solution/ranges.size();
  }
}

//-----------------------------------------------------------------------------
Eigen::Vector2d SimpleTrilateration2D::compute(const VectorOfEigenVector2d & tagPositions,
                                               const std::vector<double> & ranges,
                                               const std::vector<size_t> & rangesIndexes)
{
  assert(tagPositions.size() >= 2);
  assert(rangesIndexes.size() >= 2);
  assert(tagPositions.size() == ranges.size());

  if (tagPositions.size() == 2)
  {
    return compute_(tagPositions, ranges, rangesIndexes, 0, 1);
  } else {
    Eigen::Vector2d solution = Eigen::Vector2d::Zero();
    for (size_t i=0, j=1; i < rangesIndexes.size(); ++i, j=(j+1)%rangesIndexes.size())
    {
      solution+=compute_(tagPositions,
                         ranges,
                         rangesIndexes,
                         i,
                         j);
    }
    return solution/ranges.size();
  }
}

}  // namespace romea

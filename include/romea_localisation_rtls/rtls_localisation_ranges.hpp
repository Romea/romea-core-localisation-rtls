#ifndef __RTLSLocalisationRangeInfo_HPP__
#define __RTLSLocalisationRangeInfo_HPP__


//romea
#include <romea_rtls/RTLSRange.hpp>
#include <romea_common/pointset/PointSet.hpp>

//Eigen
#include <Eigen/Core>

//std
#include <optional>
#include <fstream>
#include <vector>

namespace romea
{

class RTLSLocalisationRange
{

public :

  RTLSLocalisationRange(const int & initiatorId,
                        const Eigen::Vector3d & initiatorPosition,
                        const int & responderId,
                        const Eigen::Vector3d & responderPosition,
                        const double & bias);

  bool isAvailable()const;

  const RTLSRange & get()const;

  void set(const RTLSRange & range);

  void reset();


  double computeUnbiasedRange3D()const;

  double computeUnbiasedRange2D()const;

  void log()const;

private :

  std::optional<RTLSRange> range_;

  double bias_;
  double zOffset_;
  double squareZOffset_;

  mutable std::ofstream logFile_;
};


using RTLSLocalisationRangeVector = std::vector<RTLSLocalisationRange>;
using RTLSLocalisationRangeArray = std::vector<RTLSLocalisationRangeVector>;

}
#endif

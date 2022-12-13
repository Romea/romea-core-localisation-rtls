#ifndef ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_RANGES_HPP_ 
#define ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_RANGES_HPP_ 


//std
#include <optional>
#include <fstream>
#include <vector>

//Eigen
#include <Eigen/Core>

//romea
#include <romea_core_rtls/RTLSRange.hpp>
#include <romea_core_common/pointset/PointSet.hpp>

namespace romea
{


// TODO(jean) change _name
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

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_RANGES_HPP_

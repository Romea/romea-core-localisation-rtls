#ifndef ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_POSITION_ESTIMATOR_HPP_ 
#define ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_POSITION_ESTIMATOR_HPP_ 

// std
#include <vector>

// romea
#include <romea_core_common/regression/leastsquares/NLSE.hpp>
#include <romea_core_common/containers/Eigen/VectorOfEigenVector.hpp>
#include "romea_core_localisation_rtls/rtls_localisation_ranges.hpp"

namespace romea
{

class RTLSLocalisationPositionEstimator : public NLSE<double>
{
public:
  RTLSLocalisationPositionEstimator();

  explicit RTLSLocalisationPositionEstimator(const double & estimateEpsilon);

  virtual ~RTLSLocalisationPositionEstimator() = default;

public :

  void loadGeometry(const VectorOfEigenVector3d &referenceTagPositions);

  bool init(const RTLSLocalisationRangeVector & ranges);

private :

  void computeGuess_()override;

  void computeJacobianAndY_()override;

private :

  VectorOfEigenVector2d referenceTagPositions_;
  std::vector<size_t> indexesOfAvailableRanges_;
  std::vector<double> ranges_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_POSITION_ESTIMATOR_HPP_ 



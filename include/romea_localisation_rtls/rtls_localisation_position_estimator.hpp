#ifndef _romea_R2HPositionEstimator_hpp
#define _romea_R2HPositionEstimator_hpp

//romea
#include <romea_common/regression/leastsquares/NLSE.hpp>
#include <romea_common/containers/Eigen/VectorOfEigenVector.hpp>
#include "rtls_localisation_ranges.hpp"

namespace romea
{

class RTLSLocalisationPositionEstimator : public NLSE<double>
{

public:

  RTLSLocalisationPositionEstimator();

  RTLSLocalisationPositionEstimator(const double & estimateEpsilon);

  virtual ~RTLSLocalisationPositionEstimator()=default;

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

}

#endif



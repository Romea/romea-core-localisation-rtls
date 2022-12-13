#ifndef _ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_POSE_ESTIMATOR_HPP
#define _ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_POSE_ESTIMATOR_HPP

// std
#include <vector>

// romea
#include <romea_core_common/regression/leastsquares/NLSE.hpp>
#include <romea_core_common/containers/Eigen/VectorOfEigenVector.hpp>
#include <romea_core_localisation_rtls/rtls_localisation_ranges.hpp>

namespace romea
{

class RTLSLocalisationPoseEstimator : public NLSE<double>
{
public:
  RTLSLocalisationPoseEstimator();

  explicit RTLSLocalisationPoseEstimator(const double & estimateEpsilon);

  virtual ~RTLSLocalisationPoseEstimator() = default;

  void loadGeometry(const VectorOfEigenVector3d & targetTagPositions,
                    const VectorOfEigenVector3d & referenceTagPositions);

  bool initR2R(const RTLSLocalisationRangeArray &ranges);

  bool initR2W(const RTLSLocalisationRangeArray &ranges,
               const std::vector<size_t> & referenceTagIndexes);

private :

  void computeGuess_()override;

  void computeJacobianAndY_()override;

private :


  VectorOfEigenVector2d targetTagPositions_;
  VectorOfEigenVector2d referenceTagPositions_;
  std::vector<std::vector<double>> ranges_;
  std::vector<std::vector<size_t>> indexesOfAvailableRanges_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_POSE_ESTIMATOR_HPP



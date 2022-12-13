#include "romea_core_localisation_rtls/rtls_localisation_position_estimator.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_simple_trilateration.hpp"
#include "romea_core_localisation_rtls/rtls_localisation_ranges.hpp"

namespace
{
const size_t MINIMAL_NUMBER_OF_RANGES_TO_COMPUTE_POSITION = 2;
}

namespace romea {

//-----------------------------------------------------------------------------
RTLSLocalisationPositionEstimator::RTLSLocalisationPositionEstimator()
{
  estimate_.resize(2);
  estimateCovariance_.resize(2, 2);
  leastSquares_.setEstimateSize(2);
}

//-----------------------------------------------------------------------------
RTLSLocalisationPositionEstimator::
RTLSLocalisationPositionEstimator(const double & estimateEpsilon):
  NLSE(estimateEpsilon)
{
  estimate_.resize(2);
  estimateCovariance_.resize(2, 2);
  leastSquares_.setEstimateSize(2);
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPositionEstimator::
loadGeometry(const VectorOfEigenVector3d &referenceTagPositions)
{
  ranges_.resize(referenceTagPositions.size());
  referenceTagPositions_.resize(referenceTagPositions.size());

  for (size_t n=0; n < referenceTagPositions.size(); ++n)
  {
    referenceTagPositions_[n]= referenceTagPositions[n].head<2>();
  }
}

//--------------------------------------- --------------------------------------
bool RTLSLocalisationPositionEstimator::init(const RTLSLocalisationRangeVector &ranges)
{
  if (ranges_.empty())
  {
    return false;
  }

  indexesOfAvailableRanges_.clear();
  for (size_t n=0; n < ranges.size() ; ++n)
  {
    if (ranges[n].isAvailable())
    {
      indexesOfAvailableRanges_.push_back(n);
      ranges_[n] = ranges[n].computeUnbiasedRange2D();
    }
  }

  if (indexesOfAvailableRanges_.size() == ranges_.size() ||
      indexesOfAvailableRanges_.size() > MINIMAL_NUMBER_OF_RANGES_TO_COMPUTE_POSITION)
  {
    leastSquares_.setDataSize(indexesOfAvailableRanges_.size());
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPositionEstimator::computeGuess_()
{
  estimate_ = SimpleTrilateration2D::
    compute(referenceTagPositions_, ranges_, indexesOfAvailableRanges_);
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPositionEstimator::computeJacobianAndY_()
{
  auto & J = leastSquares_.getJ();
  auto & Y = leastSquares_.getY();

  for (size_t n=0; n < indexesOfAvailableRanges_.size();++n)
  {
    size_t rangeIndex = indexesOfAvailableRanges_[n];
    double dx = estimate_(0) - referenceTagPositions_[rangeIndex].x();
    double dy = estimate_(1) - referenceTagPositions_[rangeIndex].y();

    Y(int(n)) = std::sqrt(dx*dx+dy*dy);

    J.row(int(n)) << dx,dy;
    J.row(int(n)) /= Y(int(n));

    Y(int(n)) -= ranges_[rangeIndex];
  }
}

}  // namespace romea

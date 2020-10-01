#include "romea_localisation_rtls/rtls_localisation_ranges.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
RTLSLocalisationRange::RTLSLocalisationRange(const int & initiatorId,
                                             const Eigen::Vector3d & initiatorPosition,
                                             const int & responderId,
                                             const Eigen::Vector3d & responderPosition,
                                             const double & bias):
  range_(),
  bias_(bias),
  zOffset_(initiatorPosition.z()-responderPosition.z()),
  squareZOffset_(zOffset_*zOffset_),
  logFile_()
{
//  std::string filename = getPrivateNodeHandle().getNamespace()+"/debug.dat";
//  std::replace_copy(std::begin(filename)+1, std::end(filename), std::begin(filename)+1, '/', '-');
//  debugLogger_.init(ros::file_log::getLogDirectory()+filename);

  std::string filename =  "range_"+std::to_string(initiatorId)+"_"+std::to_string(responderId);
  //  logFile.open(join(log_directory_,filename)+".dat");
  logFile_.open(filename+".dat");
}

//-----------------------------------------------------------------------------
const RTLSRange & RTLSLocalisationRange::get()const
{
  return range_.get();
}

//-----------------------------------------------------------------------------
bool RTLSLocalisationRange::isAvailable()const
{
  return range_.is_initialized();
}

//-----------------------------------------------------------------------------
void RTLSLocalisationRange::set(const RTLSRange & range)
{
  range_=range;
}

//-----------------------------------------------------------------------------
void RTLSLocalisationRange::reset()
{
  range_.reset();
}

//-----------------------------------------------------------------------------
double RTLSLocalisationRange::computeUnbiasedRange3D()const
{
  assert(range_.is_initialized());
  assert(range_->range-bias_>0);
  return range_->range-bias_;
}

//-----------------------------------------------------------------------------
double  RTLSLocalisationRange::computeUnbiasedRange2D()const
{
  double range = computeUnbiasedRange3D();
  assert(range*range-squareZOffset_>0);
  return std::sqrt((range*range)-squareZOffset_);
}

//-----------------------------------------------------------------------------
void RTLSLocalisationRange::log()const
{
  assert(logFile_.is_open());
  if(range_.is_initialized())
  {
    logFile_ << range_->duration.count() << " ";
    logFile_ << range_->range << " ";
    logFile_ << int(range_->firstPathRxPowerLevel) << " ";
    logFile_ << int(range_->totalRxPowerLevel) << std::endl;;
  }
}



}

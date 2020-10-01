#ifndef __RTLSLocalisationTransceivers_HPP__
#define __RTLSLocalisationTransceivers_HPP__

//std
#include <vector>

//romea
#include <romea_common/pointset/PointSet.hpp>

namespace romea
{

struct RTLSLocalisationTransceivers
{
  RTLSLocalisationTransceivers();

  size_t poll_index;
  PointSet<Eigen::Vector3d> positions;
  std::vector<std::string> frame_ids;
  std::vector<int> ids;
};

}


#endif

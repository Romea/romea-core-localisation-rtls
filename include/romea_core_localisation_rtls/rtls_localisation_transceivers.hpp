#ifndef ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_TRANSCEIVERS_HPP_ 
#define ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_TRANSCEIVERS_HPP_ 

// std
#include <vector>
#include <string>

// romea
#include <romea_core_common/pointset/PointSet.hpp>

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

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS_RTLS_LOCALISATION_TRANSCEIVERS_HPP_ 

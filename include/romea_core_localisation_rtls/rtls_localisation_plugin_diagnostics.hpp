#ifndef __RTLSLocalisationMasterDiagnostics_HPP__
#define __RTLSLocalisationMasterDiagnostics_HPP__

//std
#include <atomic>
#include <memory>

//romea
#include <romea_core_common/monitoring/OnlineAverage.hpp>
#include <romea_core_common/monitoring/RateMonitoring.hpp>
#include <romea_core_common/diagnostic/CheckupReliability.hpp>

namespace romea {

class RTLSLocalisationPluginDiagnostics
{

public:

  RTLSLocalisationPluginDiagnostics();

public :

  void setPollRate(const double & poll_rate);

  void setResponderIds(const std::vector<int> & leader_initiator_ids);

  void setInitiatorIds(const std::vector<int> & follower_initiator_ids);

  void updateInitiatorReliability(const double & reliability, const size_t & initiator_index);

  void updateResponderReliability(const double & reliability, const size_t & responder_index);

  DiagnosticReport makeReport()const;

private :

  double poll_rate_;
  std::vector<std::unique_ptr<OnlineAverage> > responder_reliability_monitorings_;
  std::vector<std::unique_ptr<CheckupReliability> > responder_reliability_diagnostics_;
  std::vector<std::unique_ptr<OnlineAverage> > initiator_reliability_monitorings_;
  std::vector<std::unique_ptr<CheckupReliability> > initiator_reliability_diagnostics_;

};

}
#endif

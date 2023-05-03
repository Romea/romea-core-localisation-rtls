// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_PLUGIN_DIAGNOSTICS_HPP_
#define ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_PLUGIN_DIAGNOSTICS_HPP_

// std
#include <atomic>
#include <memory>
#include <vector>

// romea
#include "romea_core_common/monitoring/OnlineAverage.hpp"
#include "romea_core_common/monitoring/RateMonitoring.hpp"
#include "romea_core_common/diagnostic/CheckupReliability.hpp"

namespace romea
{

class RTLSLocalisationPluginDiagnostics
{
public:
  RTLSLocalisationPluginDiagnostics();

public:
  void setPollRate(const double & poll_rate);

  void setResponderIds(const std::vector<int> & leader_initiator_ids);

  void setInitiatorIds(const std::vector<int> & follower_initiator_ids);

  void updateInitiatorReliability(const double & reliability, const size_t & initiator_index);

  void updateResponderReliability(const double & reliability, const size_t & responder_index);

  DiagnosticReport makeReport()const;

private:
  double poll_rate_;
  std::vector<std::unique_ptr<OnlineAverage>> responder_reliability_monitorings_;
  std::vector<std::unique_ptr<CheckupReliability>> responder_reliability_diagnostics_;
  std::vector<std::unique_ptr<OnlineAverage>> initiator_reliability_monitorings_;
  std::vector<std::unique_ptr<CheckupReliability>> initiator_reliability_diagnostics_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_RTLS__RTLS_LOCALISATION_PLUGIN_DIAGNOSTICS_HPP_

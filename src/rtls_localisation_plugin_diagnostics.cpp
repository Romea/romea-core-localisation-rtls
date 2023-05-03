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

// std
#include <memory>
#include <string>
#include <utility>
#include <vector>

// romea
#include "romea_core_localisation_rtls/rtls_localisation_plugin_diagnostics.hpp"

namespace
{
const double DEFAULT_LOW_RELIABILITY_THRESHOLD = 0.3;
const double DEFAULT_HIGH_RELIABILITY_THRESHOLD = 0.8;
const double MAXIMAL_NUMBER_OF_FOLLOWER_INITIATORS = 4;
const double MAXIMAL_NUMBER_OF_LEADER_INITIATORS = 4;
const double AVERAGE_MONITORING_PRECISION = 0.0001;
}

namespace romea
{

//-----------------------------------------------------------------------------
RTLSLocalisationPluginDiagnostics::RTLSLocalisationPluginDiagnostics()
: poll_rate_(-1),
  responder_reliability_monitorings_(),
  responder_reliability_diagnostics_(),
  initiator_reliability_monitorings_(),
  initiator_reliability_diagnostics_()
{
  responder_reliability_monitorings_.reserve(MAXIMAL_NUMBER_OF_LEADER_INITIATORS);
  initiator_reliability_monitorings_.reserve(MAXIMAL_NUMBER_OF_FOLLOWER_INITIATORS);
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPluginDiagnostics::setPollRate(const double & poll_rate)
{
  poll_rate_ = poll_rate;
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPluginDiagnostics::setResponderIds(
  const std::vector<int> & leader_initiator_ids)
{
  responder_reliability_monitorings_.clear();
  responder_reliability_diagnostics_.clear();

  size_t leader_initiator_monitorings_window_size = 2 * poll_rate_ / leader_initiator_ids.size();

  for (const int & leader_initiator_id : leader_initiator_ids) {
    std::string diagnostic_name = std::string("/leader_initiator") +
      std::to_string(leader_initiator_id);

    auto monitoring = std::make_unique<OnlineAverage>(
      AVERAGE_MONITORING_PRECISION,
      leader_initiator_monitorings_window_size);

    auto diagnostic = std::make_unique<CheckupReliability>(
      diagnostic_name,
      DEFAULT_LOW_RELIABILITY_THRESHOLD,
      DEFAULT_HIGH_RELIABILITY_THRESHOLD);

    responder_reliability_monitorings_.push_back(std::move(monitoring));
    responder_reliability_diagnostics_.push_back(std::move(diagnostic));
  }
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPluginDiagnostics::setInitiatorIds(
  const std::vector<int> & follower_initiator_ids)
{
  initiator_reliability_monitorings_.clear();
  initiator_reliability_diagnostics_.clear();

  size_t follower_initiator_monitorings_window_size =
    2 * poll_rate_ / follower_initiator_ids.size();

  for (const int & follower_initiator_id : follower_initiator_ids) {
    std::string diagnostic_name = std::string("/follower_initiator") +
      std::to_string(follower_initiator_id);

    auto monitoring = std::make_unique<OnlineAverage>(
      AVERAGE_MONITORING_PRECISION,
      follower_initiator_monitorings_window_size);

    auto diagnostic = std::make_unique<CheckupReliability>(
      diagnostic_name,
      DEFAULT_LOW_RELIABILITY_THRESHOLD,
      DEFAULT_HIGH_RELIABILITY_THRESHOLD);

    initiator_reliability_monitorings_.push_back(std::move(monitoring));
    initiator_reliability_diagnostics_.push_back(std::move(diagnostic));
  }
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPluginDiagnostics::updateInitiatorReliability(
  const double & reliability,
  const size_t & initiator_index)
{
  initiator_reliability_monitorings_[initiator_index]->update(reliability);
  initiator_reliability_diagnostics_[initiator_index]->evaluate(
    initiator_reliability_monitorings_[initiator_index]->getAverage());
}

//-----------------------------------------------------------------------------
void RTLSLocalisationPluginDiagnostics::updateResponderReliability(
  const double & reliability,
  const size_t & responder_index)
{
  responder_reliability_monitorings_[responder_index]->update(reliability);
  responder_reliability_diagnostics_[responder_index]->evaluate(
    responder_reliability_monitorings_[responder_index]->getAverage());
}

//-----------------------------------------------------------------------------
DiagnosticReport RTLSLocalisationPluginDiagnostics::makeReport()const
{
  DiagnosticReport report;
  for (const auto & diagnostic : initiator_reliability_diagnostics_) {
    report += diagnostic->getReport();
  }
  for (const auto & diagnostic : responder_reliability_diagnostics_) {
    report += diagnostic->getReport();
  }
  return report;
}

}  // namespace romea

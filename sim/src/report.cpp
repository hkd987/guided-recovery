#include "sim/report.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace sim {

Summary summarize(const std::vector<RunResult>& runs, double radius) {
  Summary s;
  s.radius = radius;
  std::vector<std::pair<double, uint64_t>> misses;
  double confirmSum = 0.0;
  int confirmN = 0;
  double dH = 0.0, dD = 0.0;
  for (const auto& r : runs) {
    ++s.runs;
    if (r.landed) ++s.landed;
    if (r.crashed) ++s.crashed;
    if (r.timedOut) ++s.timedOut;
    if (r.navBeforeCanopy) ++s.navBeforeCanopy;
    if (!r.canopyConfirmed) ++s.notConfirmed;
    if (r.canopyConfirmed && r.confirmDelay >= 0.0) {
      confirmSum += r.confirmDelay;
      ++confirmN;
      s.confirmDelayMax = std::max(s.confirmDelayMax, r.confirmDelay);
    }
    dH += r.deployH;
    dD += r.deployDist;
    if (r.fault != FaultType::None) {
      FaultStats& f = s.faults[r.faultLabel()];
      ++f.runs;
      if (r.faultExercised) {
        ++f.exercised;
        if (r.expectationMet) ++f.met;
      }
    }
    if (expectedOutcome(r.fault, r.baro2, r.gps2) == Expectation::Land && r.fault != FaultType::BaroDriftDrogue) {
      misses.emplace_back(r.landed ? r.miss : 1e9, r.seed);
      const auto ph = static_cast<gr::Phase>(r.finalPhase);
      if (r.fault == FaultType::None && (ph == gr::Phase::Spiral || ph == gr::Phase::Neutral)) ++s.unexpectedFallback;
      if (r.fault == FaultType::None && r.faults != 0) {
        // Count runs that latched a fault even if they later landed.
        if (ph == gr::Phase::Landed) ++s.unexpectedFallback;
      }
    }
  }
  if (confirmN) s.confirmDelayMean = confirmSum / confirmN;
  if (s.runs) {
    s.meanDeployH = dH / s.runs;
    s.meanDeployDist = dD / s.runs;
  }
  if (!misses.empty()) {
    std::sort(misses.begin(), misses.end());
    int within = 0;
    double sum = 0.0;
    for (const auto& m : misses) {
      if (m.first <= radius) ++within;
      sum += std::min(m.first, 5000.0);
    }
    const size_t n = misses.size();
    s.withinPct = 100.0 * within / static_cast<double>(n);
    s.cep50 = misses[n / 2].first;
    s.p90 = misses[std::min(n - 1, static_cast<size_t>(0.9 * static_cast<double>(n)))].first;
    s.meanMiss = sum / static_cast<double>(n);
    s.maxMiss = misses.back().first;
    for (size_t i = 0; i < std::min<size_t>(10, n); ++i) s.worstSeeds.push_back(misses[n - 1 - i].second);
  }
  return s;
}

void writeRunsCsv(const std::string& path, const std::vector<RunResult>& runs) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) return;
  std::fprintf(f,
               "seed,fault,dual_deploy,wind_ref,apogee,deploy_h,deploy_dist,landed,land_n,land_e,miss,flight_time,"
               "canopy_confirmed,confirm_delay,nav_before_canopy,crashed,timed_out,final_phase,faults,"
               "fault_exercised,fallback_delay,expectation_met,max_canopy_descent,note,true_trim,launch_delay\n");
  for (const auto& r : runs) {
    std::fprintf(f, "%llu,%s,%d,%.2f,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%d,%d,%d,%s,%u,%d,%.2f,%d,%.2f,%s,%.3f,%.2f\n",
                 static_cast<unsigned long long>(r.seed), r.faultLabel().c_str(), r.dualDeploy ? 1 : 0, r.windRef,
                 r.apogee, r.deployH, r.deployDist, r.landed ? 1 : 0, r.landN, r.landE, r.miss, r.flightTime,
                 r.canopyConfirmed ? 1 : 0, r.confirmDelay, r.navBeforeCanopy ? 1 : 0, r.crashed ? 1 : 0,
                 r.timedOut ? 1 : 0, gr::phaseName(static_cast<gr::Phase>(r.finalPhase)), r.faults,
                 r.faultExercised ? 1 : 0, r.fallbackDelay, r.expectationMet ? 1 : 0, r.maxCanopyDescent,
                 r.note.c_str(), r.trueTrim, r.launchDelay);
  }
  std::fclose(f);
}

std::string summaryJson(const Summary& s) {
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(3);
  o << "{\"runs\":" << s.runs << ",\"landed\":" << s.landed << ",\"crashed\":" << s.crashed
    << ",\"timed_out\":" << s.timedOut << ",\"nav_before_canopy\":" << s.navBeforeCanopy
    << ",\"not_confirmed\":" << s.notConfirmed << ",\"unexpected_fallback\":" << s.unexpectedFallback
    << ",\"radius_m\":" << s.radius << ",\"within_pct\":" << s.withinPct << ",\"cep50_m\":" << s.cep50
    << ",\"p90_m\":" << s.p90 << ",\"mean_miss_m\":" << s.meanMiss << ",\"max_miss_m\":" << s.maxMiss
    << ",\"confirm_delay_mean_s\":" << s.confirmDelayMean << ",\"confirm_delay_max_s\":" << s.confirmDelayMax
    << ",\"mean_deploy_h_m\":" << s.meanDeployH << ",\"mean_deploy_dist_m\":" << s.meanDeployDist
    << ",\"worst_seeds\":[";
  for (size_t i = 0; i < s.worstSeeds.size(); ++i) o << (i ? "," : "") << s.worstSeeds[i];
  o << "],\"faults\":{";
  bool first = true;
  for (const auto& kv : s.faults) {
    o << (first ? "" : ",") << "\"" << kv.first << "\":{\"runs\":" << kv.second.runs
      << ",\"exercised\":" << kv.second.exercised << ",\"met\":" << kv.second.met << "}";
    first = false;
  }
  o << "}}";
  return o.str();
}

void printSummary(const char* title, const Summary& s) {
  std::printf("== %s ==\n", title);
  std::printf("  runs %d, landed %d, crashed %d, timed out %d\n", s.runs, s.landed, s.crashed, s.timedOut);
  std::printf("  within %.0f m: %.1f%%   CEP50 %.1f m   p90 %.1f m   mean %.1f m   max %.1f m\n", s.radius,
              s.withinPct, s.cep50, s.p90, s.meanMiss, s.maxMiss);
  std::printf("  canopy not confirmed: %d   nav before canopy: %d   unexpected fallback: %d\n", s.notConfirmed,
              s.navBeforeCanopy, s.unexpectedFallback);
  std::printf("  confirm delay mean %.2f s, max %.2f s   mean deploy %.0f m AGL, %.0f m from pad\n",
              s.confirmDelayMean, s.confirmDelayMax, s.meanDeployH, s.meanDeployDist);
  for (const auto& kv : s.faults)
    std::printf("  fault %-12s runs %4d  exercised %4d  expected outcome %4d/%d\n", kv.first.c_str(), kv.second.runs,
                kv.second.exercised, kv.second.met, kv.second.exercised);
}

}  // namespace sim

// Monte Carlo summaries, gate checks and machine-readable reports.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "sim/runners.hpp"

namespace sim {

struct FaultStats {
  int runs = 0;
  int exercised = 0;
  int met = 0;
};

struct Summary {
  int runs = 0;
  int landed = 0;
  int crashed = 0;
  int timedOut = 0;
  int navBeforeCanopy = 0;
  int notConfirmed = 0;
  int unexpectedFallback = 0;  // nominal runs that ended in spiral/neutral
  double radius = 30.0;
  double withinPct = 0.0;      // % of nominal runs landing within radius
  double cep50 = 0.0, p90 = 0.0, meanMiss = 0.0, maxMiss = 0.0;
  double confirmDelayMean = 0.0, confirmDelayMax = 0.0;
  double meanDeployH = 0.0, meanDeployDist = 0.0;
  std::vector<uint64_t> worstSeeds;
  std::map<std::string, FaultStats> faults;
};

// Accuracy statistics use only runs without a fault that should end in a
// spiral (nominal, gust and shear runs).
Summary summarize(const std::vector<RunResult>& runs, double radius = 30.0);
void writeRunsCsv(const std::string& path, const std::vector<RunResult>& runs);
std::string summaryJson(const Summary& s);
void printSummary(const char* title, const Summary& s);

}  // namespace sim

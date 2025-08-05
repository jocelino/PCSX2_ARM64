// ARM64 Optimization Tracker Header
// Tracks optimization effectiveness and performance gains

#pragma once

#include <string>
#include <cstdint>

namespace ARM64Recompiler {

struct OptimizationReport {
    std::string name;
    uint64_t applications;
    uint64_t totalTimeBefore;
    uint64_t totalTimeAfter;
    double averageSpeedup;
    uint64_t timeSaved;
};

// Optimization tracking functions
void trackOptimization(const std::string& name, uint64_t beforeTime, uint64_t afterTime);
OptimizationReport getOptimizationReport(const std::string& name);
void clearOptimizationData();

} // namespace ARM64Recompiler
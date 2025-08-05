// ARM64 Optimization Tracker Implementation
// Tracks optimization effectiveness and performance gains

#include "OptimizationTracker.h"
#include <unordered_map>
#include <mutex>

namespace ARM64Recompiler {

struct OptimizationData {
    std::string name;
    uint64_t beforeTime;
    uint64_t afterTime;
    uint64_t applications;
    double totalSpeedup;
};

class OptimizationTracker {
private:
    std::unordered_map<std::string, OptimizationData> optimizations;
    std::mutex trackerMutex;
    
public:
    void recordOptimization(const std::string& name, uint64_t beforeTime, uint64_t afterTime) {
        std::lock_guard<std::mutex> lock(trackerMutex);
        auto& data = optimizations[name];
        data.name = name;
        data.beforeTime += beforeTime;
        data.afterTime += afterTime;
        data.applications++;
        
        if (beforeTime > 0) {
            double speedup = static_cast<double>(beforeTime) / afterTime;
            data.totalSpeedup += speedup;
        }
    }
    
    OptimizationReport getReport(const std::string& name) {
        std::lock_guard<std::mutex> lock(trackerMutex);
        auto it = optimizations.find(name);
        if (it == optimizations.end()) {
            return OptimizationReport{};
        }
        
        const auto& data = it->second;
        OptimizationReport report;
        report.name = data.name;
        report.applications = data.applications;
        report.totalTimeBefore = data.beforeTime;
        report.totalTimeAfter = data.afterTime;
        report.averageSpeedup = (data.applications > 0) ? data.totalSpeedup / data.applications : 1.0;
        report.timeSaved = data.beforeTime - data.afterTime;
        return report;
    }
    
    void clearOptimizations() {
        std::lock_guard<std::mutex> lock(trackerMutex);
        optimizations.clear();
    }
};

static OptimizationTracker globalTracker;

void trackOptimization(const std::string& name, uint64_t beforeTime, uint64_t afterTime) {
    globalTracker.recordOptimization(name, beforeTime, afterTime);
}

OptimizationReport getOptimizationReport(const std::string& name) {
    return globalTracker.getReport(name);
}

void clearOptimizationData() {
    globalTracker.clearOptimizations();
}

} // namespace ARM64Recompiler
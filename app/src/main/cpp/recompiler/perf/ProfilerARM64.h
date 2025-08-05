// ARM64 Profiler Header
// Performance profiling utilities for ARM64

#pragma once

#include <string>
#include <cstdint>

namespace ARM64Recompiler {

struct ProfileStats {
    uint64_t totalTime;
    uint64_t callCount;
    uint64_t averageTime;
    uint64_t minTime;
    uint64_t maxTime;
};

class ProfileTimer {
private:
    std::string profileName;
    uint64_t startTime;
    
public:
    ProfileTimer(const std::string& name);
    ~ProfileTimer();
};

// Profiling functions
uint64_t getHighResolutionTime();
ProfileStats getProfileStats(const std::string& name);
void clearAllProfiles();

// Macro for easy profiling
#define PROFILE_SCOPE(name) ARM64Recompiler::ProfileTimer timer(name)

} // namespace ARM64Recompiler
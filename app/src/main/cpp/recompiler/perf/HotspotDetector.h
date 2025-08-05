// ARM64 Hotspot Detector Header
// Identifies performance hotspots for optimization

#pragma once

#include <cstdint>
#include <vector>

namespace ARM64Recompiler {

struct HotspotInfo {
    uint32_t pc;
    uint64_t executionCount;
    uint64_t totalCycles;
    uint64_t averageCycles;
};

// Hotspot detection functions
void recordHotspot(uint32_t pc, uint64_t cycles);
std::vector<HotspotInfo> getTopHotspots(size_t count = 10);
void clearHotspotData();

} // namespace ARM64Recompiler
// ARM64 Hotspot Detector Implementation
// Identifies performance hotspots for optimization

#include "HotspotDetector.h"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <mutex>

namespace ARM64Recompiler {

struct HotspotData {
    uint32_t pc;
    uint64_t executionCount;
    uint64_t totalCycles;
};

class HotspotDetector {
private:
    std::unordered_map<uint32_t, HotspotData> hotspots;
    std::mutex hotspotMutex;
    
public:
    void recordExecution(uint32_t pc, uint64_t cycles) {
        std::lock_guard<std::mutex> lock(hotspotMutex);
        auto& data = hotspots[pc];
        data.pc = pc;
        data.executionCount++;
        data.totalCycles += cycles;
    }
    
    std::vector<HotspotInfo> getTopHotspots(size_t count) {
        std::lock_guard<std::mutex> lock(hotspotMutex);
        
        std::vector<HotspotData> sorted;
        for (const auto& pair : hotspots) {
            sorted.push_back(pair.second);
        }
        
        std::sort(sorted.begin(), sorted.end(), 
                 [](const HotspotData& a, const HotspotData& b) {
                     return a.totalCycles > b.totalCycles;
                 });
        
        std::vector<HotspotInfo> result;
        size_t limit = std::min(count, sorted.size());
        for (size_t i = 0; i < limit; i++) {
            HotspotInfo info;
            info.pc = sorted[i].pc;
            info.executionCount = sorted[i].executionCount;
            info.totalCycles = sorted[i].totalCycles;
            info.averageCycles = sorted[i].totalCycles / sorted[i].executionCount;
            result.push_back(info);
        }
        
        return result;
    }
    
    void clearHotspots() {
        std::lock_guard<std::mutex> lock(hotspotMutex);
        hotspots.clear();
    }
};

static HotspotDetector globalDetector;

void recordHotspot(uint32_t pc, uint64_t cycles) {
    globalDetector.recordExecution(pc, cycles);
}

std::vector<HotspotInfo> getTopHotspots(size_t count) {
    return globalDetector.getTopHotspots(count);
}

void clearHotspotData() {
    globalDetector.clearHotspots();
}

} // namespace ARM64Recompiler
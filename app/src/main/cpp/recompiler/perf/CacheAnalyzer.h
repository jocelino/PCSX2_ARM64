// ARM64 Cache Analyzer Header
// Analyzes cache performance and optimization opportunities

#pragma once

#include <cstdint>

namespace ARM64Recompiler {

struct CacheAnalysisReport {
    uint64_t hits;
    uint64_t misses;
    uint64_t evictions;
    uint64_t totalAccesses;
    double hitRate;
    double missRate;
};

// Cache analysis functions
void recordCacheHit();
void recordCacheMiss();
void recordCacheEviction();
CacheAnalysisReport getCacheAnalysis();
void resetCacheAnalysis();

} // namespace ARM64Recompiler
// ARM64 Optimized Instruction Cache Header
// High-performance instruction caching system

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

// Forward declaration for compiled code
struct CompiledCode {
    void* codePtr;
    uint32_t size;
    uint32_t executionCount;
};

// Cache performance statistics
struct CacheStats {
    uint64_t hitCount;
    uint64_t missCount;
    uint64_t totalLookups;
    double hitRate;
    uint32_t activeEntries;
};

// Public API for instruction cache management
CompiledCode* lookupInstruction(uint32_t pc, const uint32_t* instructions, size_t count);

void insertCompiledCode(uint32_t pc, const uint32_t* instructions, size_t count, 
                       CompiledCode* code, uint32_t codeSize);

void invalidateInstructionRange(uint32_t startPC, uint32_t endPC);

CacheStats getInstructionCacheStats();

void clearInstructionCache();

} // namespace ARM64Recompiler
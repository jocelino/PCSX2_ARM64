// ARM64 R5900 Analysis Header
// Code analysis and optimization for R5900 instruction blocks

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

enum OptimizationLevel {
    OPTIMIZATION_NONE,
    OPTIMIZATION_BASIC,
    OPTIMIZATION_MODERATE,
    OPTIMIZATION_AGGRESSIVE
};

struct BlockInfo {
    size_t instructionCount;
    bool hasLoops;
    bool hasBranches;
    bool canOptimize;
};

// Analysis functions
bool isValidR5900Instruction(uint32_t instruction);
size_t analyzeR5900Block(const uint32_t* instructions, size_t count, BlockInfo* info);
OptimizationLevel getOptimizationLevel(const BlockInfo* info);

} // namespace ARM64Recompiler
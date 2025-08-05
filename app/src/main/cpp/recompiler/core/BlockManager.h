// ARM64 Block Manager Header
// Manages compiled code blocks and cache invalidation

#pragma once

#include <cstdint>

namespace ARM64Recompiler {

// Block management functions
void* getCompiledBlock(uint32_t pc);
void addCompiledBlock(uint32_t pc, void* block);
void invalidateCompiledBlock(uint32_t pc);
void clearAllBlocks();

} // namespace ARM64Recompiler
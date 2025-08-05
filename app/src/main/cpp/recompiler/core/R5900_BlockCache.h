// ARM64 R5900 Block Cache Header
// Caches compiled R5900 instruction blocks

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

// R5900 block cache functions
void* getR5900CachedBlock(uint32_t pc);
void cacheR5900Block(uint32_t pc, void* block);
void invalidateR5900Block(uint32_t pc);
void clearR5900Cache();
size_t getR5900CacheSize();

} // namespace ARM64Recompiler
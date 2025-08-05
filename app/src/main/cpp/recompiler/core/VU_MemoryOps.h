// ARM64 VU Memory Operations Header
// Optimized memory operations for VU units

#pragma once

#include <cstddef>

namespace ARM64Recompiler {

// VU memory operation functions
void vuMemoryLoad(float* dst, const void* src, size_t elementCount);
void vuMemoryStore(void* dst, const float* src, size_t elementCount);
void vuMemoryClear(void* ptr, size_t size);

} // namespace ARM64Recompiler
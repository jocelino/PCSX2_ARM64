// ARM64 Memory Helpers Header
// Optimized memory operations for ARM64

#pragma once

#include <cstdint>

namespace ARM64Recompiler {

// Fast memory operations
void fastMemcpy16(void* dst, const void* src);
void fastMemcpy32(void* dst, const void* src);
void fastMemset16(void* dst, uint8_t value);
void fastMemset32(void* dst, uint8_t value);

// Alignment checks
bool isAligned16(const void* ptr);
bool isAligned32(const void* ptr);

// Cache management
void prefetchL1(const void* ptr);
void prefetchL2(const void* ptr);

} // namespace ARM64Recompiler
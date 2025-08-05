// ARM64 Memory Helpers Implementation
// Optimized memory operations for ARM64

#include "ARM64_MemoryHelpers.h"
#include <arm_neon.h>
#include <cstring>

namespace ARM64Recompiler {

void fastMemcpy16(void* dst, const void* src) {
    uint8x16_t data = vld1q_u8(static_cast<const uint8_t*>(src));
    vst1q_u8(static_cast<uint8_t*>(dst), data);
}

void fastMemcpy32(void* dst, const void* src) {
    uint8x16_t data1 = vld1q_u8(static_cast<const uint8_t*>(src));
    uint8x16_t data2 = vld1q_u8(static_cast<const uint8_t*>(src) + 16);
    vst1q_u8(static_cast<uint8_t*>(dst), data1);
    vst1q_u8(static_cast<uint8_t*>(dst) + 16, data2);
}

void fastMemset16(void* dst, uint8_t value) {
    uint8x16_t data = vdupq_n_u8(value);
    vst1q_u8(static_cast<uint8_t*>(dst), data);
}

void fastMemset32(void* dst, uint8_t value) {
    uint8x16_t data = vdupq_n_u8(value);
    vst1q_u8(static_cast<uint8_t*>(dst), data);
    vst1q_u8(static_cast<uint8_t*>(dst) + 16, data);
}

bool isAligned16(const void* ptr) {
    return (reinterpret_cast<uintptr_t>(ptr) & 15) == 0;
}

bool isAligned32(const void* ptr) {
    return (reinterpret_cast<uintptr_t>(ptr) & 31) == 0;
}

void prefetchL1(const void* ptr) {
    __builtin_prefetch(ptr, 0, 3); // Read, high temporal locality
}

void prefetchL2(const void* ptr) {
    __builtin_prefetch(ptr, 0, 2); // Read, moderate temporal locality
}

} // namespace ARM64Recompiler
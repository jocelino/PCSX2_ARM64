// ARM64 VU Memory Operations Implementation
// Optimized memory operations for VU units

#include "VU_MemoryOps.h"
#include <arm_neon.h>
#include <cstring>

namespace ARM64Recompiler {

void vuMemoryLoad(float* dst, const void* src, size_t elementCount) {
    const float* srcFloat = static_cast<const float*>(src);
    
    // Process 4 elements at a time with NEON
    size_t neonCount = elementCount / 4;
    for (size_t i = 0; i < neonCount; ++i) {
        float32x4_t data = vld1q_f32(srcFloat + i * 4);
        vst1q_f32(dst + i * 4, data);
    }
    
    // Handle remaining elements
    size_t remaining = elementCount % 4;
    if (remaining > 0) {
        std::memcpy(dst + neonCount * 4, srcFloat + neonCount * 4, remaining * sizeof(float));
    }
}

void vuMemoryStore(void* dst, const float* src, size_t elementCount) {
    float* dstFloat = static_cast<float*>(dst);
    
    // Process 4 elements at a time with NEON
    size_t neonCount = elementCount / 4;
    for (size_t i = 0; i < neonCount; ++i) {
        float32x4_t data = vld1q_f32(src + i * 4);
        vst1q_f32(dstFloat + i * 4, data);
    }
    
    // Handle remaining elements
    size_t remaining = elementCount % 4;
    if (remaining > 0) {
        std::memcpy(dstFloat + neonCount * 4, src + neonCount * 4, remaining * sizeof(float));
    }
}

void vuMemoryClear(void* ptr, size_t size) {
    if (size < 16) {
        std::memset(ptr, 0, size);
        return;
    }
    
    uint8_t* data = static_cast<uint8_t*>(ptr);
    const uint8x16_t zero = vdupq_n_u8(0);
    
    // Clear 16-byte chunks with NEON
    size_t chunks = size / 16;
    for (size_t i = 0; i < chunks; ++i) {
        vst1q_u8(data + i * 16, zero);
    }
    
    // Handle remaining bytes
    size_t remaining = size % 16;
    if (remaining > 0) {
        std::memset(data + chunks * 16, 0, remaining);
    }
}

} // namespace ARM64Recompiler
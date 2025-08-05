// ARM64 Optimized MicroVU Header
// High-performance VU recompiler interface

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <atomic>

namespace ARM64Recompiler {

// Forward declarations
struct VUProgram;

// VU register structure (128-bit aligned for NEON)
struct alignas(16) VURegisters {
    float vf[32][4];        // Vector float registers (VF0-VF31)
    uint32_t vi[16];        // Vector integer registers (VI0-VI15)
    uint32_t acc[4];        // Accumulator register
    uint32_t q;             // Q register
    uint32_t p;             // P register
    uint32_t r;             // R register
    uint32_t i;             // I register
    uint32_t status;        // Status flags
    uint32_t mac;           // MAC flags
    uint32_t clip;          // Clip flags
    uint32_t pc;            // Program counter
};

// VU execution statistics
struct VUStats {
    std::atomic<uint64_t> programsCompiled;
    std::atomic<uint64_t> instructionsExecuted;
    std::atomic<uint64_t> cacheHits;
    std::atomic<uint64_t> cacheMisses;
    std::atomic<uint64_t> compilationTime;
    std::atomic<uint64_t> executionTime;
};

// Public API functions
void initializeMicroVU();
void shutdownMicroVU();

// Synchronous compilation
VUProgram* compileMicroProgram(uint32_t startPC, uint32_t endPC, 
                              const uint32_t* microcode, size_t size);

// Asynchronous compilation with callback
void compileMicroProgramAsync(uint32_t startPC, uint32_t endPC, 
                             const uint32_t* microcode, size_t size,
                             std::function<void(VUProgram*)> callback);

// Execute compiled VU program
void executeMicroProgram(VUProgram* program, void* vuRegisters);

// Cache management
void clearVUProgramCache();
VUStats getMicroVUStats();

// ARM64-specific NEON operations (inline for performance)
namespace NEONOps {
    // Vector math operations using ARM64 NEON intrinsics
    inline void vectorAdd4(float* dst, const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t result = vaddq_f32(va, vb);
        vst1q_f32(dst, result);
    }
    
    inline void vectorSub4(float* dst, const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t result = vsubq_f32(va, vb);
        vst1q_f32(dst, result);
    }
    
    inline void vectorMul4(float* dst, const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t result = vmulq_f32(va, vb);
        vst1q_f32(dst, result);
    }
    
    inline void vectorMulAdd4(float* dst, const float* a, const float* b, const float* c) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t vc = vld1q_f32(c);
        float32x4_t result = vfmaq_f32(vc, va, vb); // c + (a * b)
        vst1q_f32(dst, result);
    }
    
    inline float vectorDot3(const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t mul = vmulq_f32(va, vb);
        
        // Sum first 3 elements
        float32x2_t sum = vadd_f32(vget_low_f32(mul), vdup_lane_f32(vget_high_f32(mul), 0));
        return vget_lane_f32(vpadd_f32(sum, sum), 0);
    }
    
    inline void vectorCross3(float* dst, const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        
        // Cross product: result = (a.yzx * b.zxy) - (a.zxy * b.yzx)
        float32x4_t a_yzx = vextq_f32(vextq_f32(va, va, 1), va, 3);
        float32x4_t b_zxy = vextq_f32(vextq_f32(vb, vb, 2), vb, 2);
        float32x4_t a_zxy = vextq_f32(vextq_f32(va, va, 2), va, 2);
        float32x4_t b_yzx = vextq_f32(vextq_f32(vb, vb, 1), vb, 3);
        
        float32x4_t result = vsubq_f32(vmulq_f32(a_yzx, b_zxy), vmulq_f32(a_zxy, b_yzx));
        vst1q_f32(dst, result);
    }
    
    inline void vectorNormalize3(float* dst, const float* src) {
        float32x4_t v = vld1q_f32(src);
        
        // Calculate dot product (magnitude squared)
        float32x4_t squared = vmulq_f32(v, v);
        float32x2_t sum = vadd_f32(vget_low_f32(squared), vdup_lane_f32(vget_high_f32(squared), 0));
        float magnitude_sq = vget_lane_f32(vpadd_f32(sum, sum), 0);
        
        // Calculate reciprocal square root
        float32x4_t rsqrt = vdupq_n_f32(1.0f / sqrtf(magnitude_sq));
        
        // Normalize
        float32x4_t result = vmulq_f32(v, rsqrt);
        vst1q_f32(dst, result);
    }
}

} // namespace ARM64Recompiler
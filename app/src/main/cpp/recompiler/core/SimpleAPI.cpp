// ARM64 Simple Recompiler API
// Basic optimized functions for PCSX2

#include <arm_neon.h>
#include <cstdint>

namespace ARM64Recompiler {

// Simple VIF unpacking with NEON
extern "C" {
    void vifUnpackV4_32_simple(void* dst, const void* src, size_t count) {
        const uint32_t* srcData = static_cast<const uint32_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        for (size_t i = 0; i < count; ++i) {
            uint32x4_t data = vld1q_u32(srcData + i * 4);
            vst1q_u32(dstData + i * 4, data);
        }
    }
    
    void vifUnpackV4_16_simple(void* dst, const void* src, size_t count) {
        const uint16_t* srcData = static_cast<const uint16_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        for (size_t i = 0; i < count; ++i) {
            uint16x4_t src16 = vld1_u16(srcData + i * 4);
            uint32x4_t dst32 = vmovl_u16(src16);
            vst1q_u32(dstData + i * 4, dst32);
        }
    }
    
    // Simple vector operations
    void vectorAdd4f_simple(float* result, const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t vr = vaddq_f32(va, vb);
        vst1q_f32(result, vr);
    }
    
    void vectorMul4f_simple(float* result, const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t vr = vmulq_f32(va, vb);
        vst1q_f32(result, vr);
    }
    
    float vectorDot3f_simple(const float* a, const float* b) {
        float32x4_t va = vld1q_f32(a);
        float32x4_t vb = vld1q_f32(b);
        float32x4_t vmul = vmulq_f32(va, vb);
        
        // Sum first 3 elements
        float sum = vgetq_lane_f32(vmul, 0) + vgetq_lane_f32(vmul, 1) + vgetq_lane_f32(vmul, 2);
        return sum;
    }
    
    // Performance counter
    uint64_t getPerformanceCounter() {
        uint64_t cycles;
        asm volatile("mrs %0, cntvct_el0" : "=r" (cycles));
        return cycles;
    }
}

} // namespace ARM64Recompiler
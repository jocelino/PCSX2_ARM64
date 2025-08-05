// ARM64 Vector Operations Implementation
// Optimized vector operations using NEON

#include "ARM64_VectorOps.h"
#include <arm_neon.h>

namespace ARM64Recompiler {

void vectorAdd4f(float* result, const float* a, const float* b) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vr = vaddq_f32(va, vb);
    vst1q_f32(result, vr);
}

void vectorSub4f(float* result, const float* a, const float* b) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vr = vsubq_f32(va, vb);
    vst1q_f32(result, vr);
}

void vectorMul4f(float* result, const float* a, const float* b) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vr = vmulq_f32(va, vb);
    vst1q_f32(result, vr);
}

void vectorMadd4f(float* result, const float* a, const float* b, const float* c) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vc = vld1q_f32(c);
    float32x4_t vr = vfmaq_f32(vc, va, vb); // c + (a * b)
    vst1q_f32(result, vr);
}

float vectorDot3f(const float* a, const float* b) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vmul = vmulq_f32(va, vb);
    
    // Sum first 3 elements
    float32x2_t sum = vadd_f32(vget_low_f32(vmul), vdup_lane_f32(vget_high_f32(vmul), 0));
    return vget_lane_f32(vpadd_f32(sum, sum), 0);
}

void vectorNormalize3f(float* result, const float* input) {
    float32x4_t v = vld1q_f32(input);
    
    // Calculate dot product (magnitude squared)
    float32x4_t squared = vmulq_f32(v, v);
    float32x2_t sum = vadd_f32(vget_low_f32(squared), vdup_lane_f32(vget_high_f32(squared), 0));
    float magnitude_sq = vget_lane_f32(vpadd_f32(sum, sum), 0);
    
    // Calculate reciprocal square root
    float rsqrt = 1.0f / sqrtf(magnitude_sq);
    float32x4_t vrsqrt = vdupq_n_f32(rsqrt);
    
    // Normalize
    float32x4_t normalized = vmulq_f32(v, vrsqrt);
    vst1q_f32(result, normalized);
}

} // namespace ARM64Recompiler
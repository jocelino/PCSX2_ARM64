// ARM64 NEON Vector Math Implementation
// Advanced vector mathematics using NEON instructions

#include "NEON_VectorMath.h"
#include <arm_neon.h>
#include <cmath>

namespace ARM64Recompiler {

void matrixMultiply4x4NEON(float* result, const float* a, const float* b) {
    // Load matrix A
    float32x4_t a0 = vld1q_f32(a + 0);
    float32x4_t a1 = vld1q_f32(a + 4);
    float32x4_t a2 = vld1q_f32(a + 8);
    float32x4_t a3 = vld1q_f32(a + 12);
    
    // Compute each column of result matrix
    for (int col = 0; col < 4; col++) {
        float32x4_t b_col = {b[col], b[col + 4], b[col + 8], b[col + 12]};
        
        float32x4_t result_col = vmulq_f32(a0, vdupq_laneq_f32(b_col, 0));
        result_col = vfmaq_f32(result_col, a1, vdupq_laneq_f32(b_col, 1));
        result_col = vfmaq_f32(result_col, a2, vdupq_laneq_f32(b_col, 2));
        result_col = vfmaq_f32(result_col, a3, vdupq_laneq_f32(b_col, 3));
        
        vst1q_f32(result + col * 4, result_col);
    }
}

void vectorCrossProduct(float* result, const float* a, const float* b) {
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    
    // Cross product: result = (a.yzx * b.zxy) - (a.zxy * b.yzx)
    float32x4_t a_yzx = vextq_f32(vextq_f32(va, va, 1), va, 3);
    float32x4_t b_zxy = vextq_f32(vextq_f32(vb, vb, 2), vb, 2);
    float32x4_t a_zxy = vextq_f32(vextq_f32(va, va, 2), va, 2);
    float32x4_t b_yzx = vextq_f32(vextq_f32(vb, vb, 1), vb, 3);
    
    float32x4_t cross = vsubq_f32(vmulq_f32(a_yzx, b_zxy), vmulq_f32(a_zxy, b_yzx));
    vst1q_f32(result, cross);
}

float vectorLength(const float* vector) {
    float32x4_t v = vld1q_f32(vector);
    float32x4_t squared = vmulq_f32(v, v);
    
    // Sum all components
    float32x2_t sum = vadd_f32(vget_low_f32(squared), vget_high_f32(squared));
    float total = vget_lane_f32(vpadd_f32(sum, sum), 0);
    
    return sqrtf(total);
}

void vectorNormalize(float* result, const float* vector) {
    float length = vectorLength(vector);
    if (length > 0.0f) {
        float invLength = 1.0f / length;
        float32x4_t v = vld1q_f32(vector);
        float32x4_t inv = vdupq_n_f32(invLength);
        float32x4_t normalized = vmulq_f32(v, inv);
        vst1q_f32(result, normalized);
    } else {
        result[0] = result[1] = result[2] = result[3] = 0.0f;
    }
}

void quaternionMultiply(float* result, const float* q1, const float* q2) {
    float32x4_t a = vld1q_f32(q1);
    float32x4_t b = vld1q_f32(q2);
    
    // Quaternion multiplication using NEON
    // result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    // result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    // result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    // result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    
    float32x4_t wwww = vdupq_laneq_f32(a, 3);
    float32x4_t xxxx = vdupq_laneq_f32(a, 0);
    float32x4_t yyyy = vdupq_laneq_f32(a, 1);
    float32x4_t zzzz = vdupq_laneq_f32(a, 2);
    
    float32x4_t wzyx = {b[3], b[2], b[1], b[0]}; // Reverse order
    float32x4_t zwxy = {b[2], b[3], b[0], b[1]}; // Rotated
    float32x4_t yxwz = {b[1], b[0], b[3], b[2]}; // Another rotation
    
    float32x4_t result1 = vmulq_f32(wwww, b);
    float32x4_t result2 = vmulq_f32(xxxx, wzyx);
    float32x4_t result3 = vmulq_f32(yyyy, zwxy);
    float32x4_t result4 = vmulq_f32(zzzz, yxwz);
    
    // Combine with appropriate signs
    float32x4_t signs = {1.0f, -1.0f, 1.0f, -1.0f};
    result2 = vmulq_f32(result2, signs);
    
    float32x4_t final_result = vaddq_f32(result1, result2);
    final_result = vaddq_f32(final_result, result3);
    final_result = vsubq_f32(final_result, result4);
    
    vst1q_f32(result, final_result);
}

void transformPoint(float* result, const float* point, const float* matrix) {
    float32x4_t p = vld1q_f32(point);
    
    // Load matrix rows
    float32x4_t m0 = vld1q_f32(matrix + 0);
    float32x4_t m1 = vld1q_f32(matrix + 4);
    float32x4_t m2 = vld1q_f32(matrix + 8);
    float32x4_t m3 = vld1q_f32(matrix + 12);
    
    // Transform point
    float32x4_t x = vmulq_f32(m0, vdupq_laneq_f32(p, 0));
    float32x4_t y = vfmaq_f32(x, m1, vdupq_laneq_f32(p, 1));
    float32x4_t z = vfmaq_f32(y, m2, vdupq_laneq_f32(p, 2));
    float32x4_t w = vfmaq_f32(z, m3, vdupq_laneq_f32(p, 3));
    
    vst1q_f32(result, w);
}

} // namespace ARM64Recompiler
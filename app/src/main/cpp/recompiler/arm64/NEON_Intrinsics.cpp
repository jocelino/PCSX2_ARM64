// ARM64 NEON Intrinsics Optimizations
// High-performance SIMD operations for PCSX2 components

#include "NEON_Intrinsics.h"
#include <arm_neon.h>
#include <cstring>

namespace ARM64Recompiler {
namespace NEONIntrinsics {

// Optimized matrix operations for 3D graphics
void matrixMultiply4x4(float* result, const float* a, const float* b) {
    // Load matrix A rows
    float32x4_t a0 = vld1q_f32(a + 0);
    float32x4_t a1 = vld1q_f32(a + 4);
    float32x4_t a2 = vld1q_f32(a + 8);
    float32x4_t a3 = vld1q_f32(a + 12);
    
    // Load matrix B columns and compute result
    for (int col = 0; col < 4; ++col) {
        float32x4_t b_col = {b[col], b[col + 4], b[col + 8], b[col + 12]};
        
        float32x4_t r0 = vmulq_f32(a0, vdupq_laneq_f32(b_col, 0));
        float32x4_t r1 = vfmaq_f32(r0, a1, vdupq_laneq_f32(b_col, 1));
        float32x4_t r2 = vfmaq_f32(r1, a2, vdupq_laneq_f32(b_col, 2));
        float32x4_t r3 = vfmaq_f32(r2, a3, vdupq_laneq_f32(b_col, 3));
        
        vst1q_f32(result + col * 4, r3);
    }
}

// Optimized vector transformations
void transformVertices(float* vertices, const float* matrix, size_t vertexCount) {
    // Load transformation matrix
    float32x4_t m0 = vld1q_f32(matrix + 0);
    float32x4_t m1 = vld1q_f32(matrix + 4);
    float32x4_t m2 = vld1q_f32(matrix + 8);
    float32x4_t m3 = vld1q_f32(matrix + 12);
    
    for (size_t i = 0; i < vertexCount; ++i) {
        float32x4_t vertex = vld1q_f32(vertices + i * 4);
        
        // Transform vertex: result = matrix * vertex
        float32x4_t x = vmulq_f32(m0, vdupq_laneq_f32(vertex, 0));
        float32x4_t y = vfmaq_f32(x, m1, vdupq_laneq_f32(vertex, 1));
        float32x4_t z = vfmaq_f32(y, m2, vdupq_laneq_f32(vertex, 2));
        float32x4_t w = vfmaq_f32(z, m3, vdupq_laneq_f32(vertex, 3));
        
        vst1q_f32(vertices + i * 4, w);
    }
}

// Optimized color format conversions
void convertRGBA8ToFloat(float* dst, const uint8_t* src, size_t pixelCount) {
    const float32x4_t scale = vdupq_n_f32(1.0f / 255.0f);
    
    for (size_t i = 0; i < pixelCount; ++i) {
        // Load 4 uint8 values
        uint8x8_t rgba8 = vld1_u8(src + i * 4);
        
        // Convert to uint16, then to uint32, then to float
        uint16x8_t rgba16 = vmovl_u8(rgba8);
        uint16x4_t rgba16_low = vget_low_u16(rgba16);
        uint32x4_t rgba32 = vmovl_u16(rgba16_low);
        float32x4_t rgbaFloat = vcvtq_f32_u32(rgba32);
        
        // Scale to [0, 1] range
        rgbaFloat = vmulq_f32(rgbaFloat, scale);
        
        vst1q_f32(dst + i * 4, rgbaFloat);
    }
}

void convertFloatToRGBA8(uint8_t* dst, const float* src, size_t pixelCount) {
    const float32x4_t scale = vdupq_n_f32(255.0f);
    
    for (size_t i = 0; i < pixelCount; ++i) {
        float32x4_t rgbaFloat = vld1q_f32(src + i * 4);
        
        // Scale to [0, 255] range and clamp
        rgbaFloat = vmulq_f32(rgbaFloat, scale);
        uint32x4_t rgba32 = vcvtq_u32_f32(rgbaFloat);
        
        // Convert to uint16, then to uint8
        uint16x4_t rgba16 = vqmovn_u32(rgba32);
        uint16x8_t rgba16_combined = vcombine_u16(rgba16, rgba16);
        uint8x8_t rgba8 = vqmovn_u16(rgba16_combined);
        
        // Store only first 4 bytes
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + i * 4), 
                     vreinterpret_u32_u8(rgba8), 0);
    }
}

// Optimized texture filtering
void bilinearFilter(float* dst, const float* src, int srcWidth, int srcHeight, 
                   float u, float v) {
    // Calculate integer and fractional parts
    float fu = u * srcWidth - 0.5f;
    float fv = v * srcHeight - 0.5f;
    
    int iu = static_cast<int>(fu);
    int iv = static_cast<int>(fv);
    
    float fracU = fu - iu;
    float fracV = fv - iv;
    
    // Clamp to texture bounds
    int x0 = std::max(0, std::min(iu, srcWidth - 1));
    int y0 = std::max(0, std::min(iv, srcHeight - 1));
    int x1 = std::max(0, std::min(iu + 1, srcWidth - 1));
    int y1 = std::max(0, std::min(iv + 1, srcHeight - 1));
    
    // Load texel values
    float32x4_t p00 = vld1q_f32(src + (y0 * srcWidth + x0) * 4);
    float32x4_t p01 = vld1q_f32(src + (y0 * srcWidth + x1) * 4);
    float32x4_t p10 = vld1q_f32(src + (y1 * srcWidth + x0) * 4);
    float32x4_t p11 = vld1q_f32(src + (y1 * srcWidth + x1) * 4);
    
    // Interpolate horizontally
    float32x4_t fracU_vec = vdupq_n_f32(fracU);
    float32x4_t top = vfmaq_f32(p00, vsubq_f32(p01, p00), fracU_vec);
    float32x4_t bottom = vfmaq_f32(p10, vsubq_f32(p11, p10), fracU_vec);
    
    // Interpolate vertically
    float32x4_t fracV_vec = vdupq_n_f32(fracV);
    float32x4_t result = vfmaq_f32(top, vsubq_f32(bottom, top), fracV_vec);
    
    vst1q_f32(dst, result);
}

// Optimized audio processing
void mixAudioChannels(float* output, const float* input1, const float* input2, 
                     float volume1, float volume2, size_t sampleCount) {
    float32x4_t vol1 = vdupq_n_f32(volume1);
    float32x4_t vol2 = vdupq_n_f32(volume2);
    
    size_t neonSamples = sampleCount & ~3; // Process in groups of 4
    
    for (size_t i = 0; i < neonSamples; i += 4) {
        float32x4_t in1 = vld1q_f32(input1 + i);
        float32x4_t in2 = vld1q_f32(input2 + i);
        
        // Mix with volumes: output = input1 * volume1 + input2 * volume2
        float32x4_t mixed = vfmaq_f32(vmulq_f32(in1, vol1), in2, vol2);
        
        vst1q_f32(output + i, mixed);
    }
    
    // Handle remaining samples
    for (size_t i = neonSamples; i < sampleCount; ++i) {
        output[i] = input1[i] * volume1 + input2[i] * volume2;
    }
}

void applyAudioFilter(float* samples, const float* coefficients, size_t sampleCount, size_t filterOrder) {
    // Simple FIR filter implementation with NEON
    for (size_t i = filterOrder; i < sampleCount; ++i) {
        float32x4_t sum = vdupq_n_f32(0.0f);
        
        // Process coefficients in groups of 4
        size_t neonCoeffs = (filterOrder / 4) * 4;
        for (size_t j = 0; j < neonCoeffs; j += 4) {
            float32x4_t coeff = vld1q_f32(coefficients + j);
            float32x4_t sample = vld1q_f32(samples + i - filterOrder + j);
            sum = vfmaq_f32(sum, coeff, sample);
        }
        
        // Sum the vector elements
        float32x2_t sum_pairs = vadd_f32(vget_low_f32(sum), vget_high_f32(sum));
        float result = vget_lane_f32(vpadd_f32(sum_pairs, sum_pairs), 0);
        
        // Handle remaining coefficients
        for (size_t j = neonCoeffs; j < filterOrder; ++j) {
            result += coefficients[j] * samples[i - filterOrder + j];
        }
        
        samples[i] = result;
    }
}

// Optimized memory operations
void fastMemoryCopy(void* dst, const void* src, size_t size) {
    if (size < 32) {
        std::memcpy(dst, src, size);
        return;
    }
    
    uint8_t* dstBytes = static_cast<uint8_t*>(dst);
    const uint8_t* srcBytes = static_cast<const uint8_t*>(src);
    
    // Copy 32 bytes at a time using NEON
    size_t neonSize = size & ~31;
    for (size_t i = 0; i < neonSize; i += 32) {
        uint8x16_t data1 = vld1q_u8(srcBytes + i);
        uint8x16_t data2 = vld1q_u8(srcBytes + i + 16);
        vst1q_u8(dstBytes + i, data1);
        vst1q_u8(dstBytes + i + 16, data2);
    }
    
    // Copy remaining bytes
    if (size & 31) {
        std::memcpy(dstBytes + neonSize, srcBytes + neonSize, size & 31);
    }
}

void fastMemorySet(void* dst, uint8_t value, size_t size) {
    if (size < 32) {
        std::memset(dst, value, size);
        return;
    }
    
    uint8_t* dstBytes = static_cast<uint8_t*>(dst);
    uint8x16_t valueVec = vdupq_n_u8(value);
    
    // Set 32 bytes at a time using NEON
    size_t neonSize = size & ~31;
    for (size_t i = 0; i < neonSize; i += 32) {
        vst1q_u8(dstBytes + i, valueVec);
        vst1q_u8(dstBytes + i + 16, valueVec);
    }
    
    // Set remaining bytes
    if (size & 31) {
        std::memset(dstBytes + neonSize, value, size & 31);
    }
}

// Performance measurement utilities
uint64_t getCycleCount() {
    uint64_t cycles;
    asm volatile("mrs %0, cntvct_el0" : "=r" (cycles));
    return cycles;
}

double getElapsedSeconds(uint64_t startCycles, uint64_t endCycles) {
    uint64_t frequency;
    asm volatile("mrs %0, cntfrq_el0" : "=r" (frequency));
    return static_cast<double>(endCycles - startCycles) / frequency;
}

} // namespace NEONIntrinsics
} // namespace ARM64Recompiler
// ARM64 Simple NEON Operations
// Basic NEON optimized operations for PCSX2

#include <arm_neon.h>
#include <cstring>

namespace ARM64Recompiler {

extern "C" {
    // Fast memory copy using NEON (16-byte aligned)
    void fastMemcpyNEON(void* dst, const void* src, size_t size) {
        if (size < 64) {
            std::memcpy(dst, src, size);
            return;
        }
        
        const uint8_t* srcBytes = static_cast<const uint8_t*>(src);
        uint8_t* dstBytes = static_cast<uint8_t*>(dst);
        
        // Copy 64 bytes at a time using NEON
        size_t neonSize = size & ~63;
        for (size_t i = 0; i < neonSize; i += 64) {
            uint8x16_t data1 = vld1q_u8(srcBytes + i);
            uint8x16_t data2 = vld1q_u8(srcBytes + i + 16);
            uint8x16_t data3 = vld1q_u8(srcBytes + i + 32);
            uint8x16_t data4 = vld1q_u8(srcBytes + i + 48);
            
            vst1q_u8(dstBytes + i, data1);
            vst1q_u8(dstBytes + i + 16, data2);
            vst1q_u8(dstBytes + i + 32, data3);
            vst1q_u8(dstBytes + i + 48, data4);
        }
        
        // Copy remaining bytes
        if (size & 63) {
            std::memcpy(dstBytes + neonSize, srcBytes + neonSize, size & 63);
        }
    }
    
    // Fast memory set using NEON
    void fastMemsetNEON(void* ptr, uint8_t value, size_t size) {
        if (size < 64) {
            std::memset(ptr, value, size);
            return;
        }
        
        uint8_t* bytes = static_cast<uint8_t*>(ptr);
        uint8x16_t valueVec = vdupq_n_u8(value);
        
        // Set 64 bytes at a time using NEON
        size_t neonSize = size & ~63;
        for (size_t i = 0; i < neonSize; i += 64) {
            vst1q_u8(bytes + i, valueVec);
            vst1q_u8(bytes + i + 16, valueVec);
            vst1q_u8(bytes + i + 32, valueVec);
            vst1q_u8(bytes + i + 48, valueVec);
        }
        
        // Handle remaining bytes
        if (size & 63) {
            std::memset(bytes + neonSize, value, size & 63);
        }
    }
    
    // Matrix multiply 4x4 using NEON
    void matrixMul4x4NEON(float* result, const float* a, const float* b) {
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
    
    // Color conversion RGBA8 to Float
    void convertRGBA8ToFloat_NEON(float* dst, const uint8_t* src, size_t pixelCount) {
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
    
    // Check if NEON is available (always true on ARM64)
    int isNEONAvailable() {
        return 1;
    }
    
    // Get ARM64 feature flags
    uint32_t getARM64Features() {
        uint32_t features = 0;
        features |= (1 << 0); // NEON
        features |= (1 << 1); // FP
        
        // Check for additional features if needed
        return features;
    }
}

} // namespace ARM64Recompiler
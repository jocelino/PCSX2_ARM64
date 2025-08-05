// ARM64 NEON Optimized VIF Unpacking Implementation
// High-performance data unpacking with NEON SIMD instructions

#include "VIF_UnpackNEON.h"
#include <arm_neon.h>
#include <cstring>

namespace ARM64Recompiler {

// ARM64 NEON optimized unpacking functions
class NEONVIFUnpacker {
private:
    // NEON-optimized memory prefetching
    static inline void prefetchData(const void* ptr, size_t size) {
        const char* data = static_cast<const char*>(ptr);
        for (size_t i = 0; i < size; i += 64) {
            __builtin_prefetch(data + i, 0, 3); // Read prefetch, high locality
        }
    }
    
    // NEON-optimized data alignment check
    static inline bool isAligned16(const void* ptr) {
        return (reinterpret_cast<uintptr_t>(ptr) & 15) == 0;
    }
    
    static inline bool isAligned32(const void* ptr) {
        return (reinterpret_cast<uintptr_t>(ptr) & 31) == 0;
    }
    
public:
    // V4-32: Unpack 4 32-bit values (1:1 mapping)
    static void unpackV4_32(void* dst, const void* src, size_t count, bool usn = false) {
        prefetchData(src, count * 16);
        
        const uint32_t* srcData = static_cast<const uint32_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        // Process 4 elements at a time with NEON
        size_t neonCount = count;
        for (size_t i = 0; i < neonCount; ++i) {
            uint32x4_t data = vld1q_u32(srcData + i * 4);
            vst1q_u32(dstData + i * 4, data);
        }
    }
    
    // V4-16: Unpack 4 16-bit values to 32-bit
    static void unpackV4_16(void* dst, const void* src, size_t count, bool usn = false) {
        prefetchData(src, count * 8);
        
        const uint16_t* srcData = static_cast<const uint16_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        for (size_t i = 0; i < count; ++i) {
            uint16x4_t src16 = vld1_u16(srcData + i * 4);
            
            if (usn) {
                // Unsigned extension
                uint32x4_t dst32 = vmovl_u16(src16);
                vst1q_u32(dstData + i * 4, dst32);
            } else {
                // Signed extension
                int16x4_t srcSigned = vreinterpret_s16_u16(src16);
                int32x4_t dst32 = vmovl_s16(srcSigned);
                vst1q_u32(dstData + i * 4, vreinterpretq_u32_s32(dst32));
            }
        }
    }
    
    // V4-8: Unpack 4 8-bit values to 32-bit
    static void unpackV4_8(void* dst, const void* src, size_t count, bool usn = false) {
        prefetchData(src, count * 4);
        
        const uint8_t* srcData = static_cast<const uint8_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        for (size_t i = 0; i < count; ++i) {
            uint8x8_t src8 = vld1_u8(srcData + i * 4); // Load 8 bytes but use only 4
            
            if (usn) {
                // Unsigned extension: 8->16->32
                uint16x8_t temp16 = vmovl_u8(src8);
                uint16x4_t low16 = vget_low_u16(temp16);
                uint32x4_t dst32 = vmovl_u16(low16);
                vst1q_u32(dstData + i * 4, dst32);
            } else {
                // Signed extension: 8->16->32
                int8x8_t srcSigned = vreinterpret_s8_u8(src8);
                int16x8_t temp16 = vmovl_s8(srcSigned);
                int16x4_t low16 = vget_low_s16(temp16);
                int32x4_t dst32 = vmovl_s16(low16);
                vst1q_u32(dstData + i * 4, vreinterpretq_u32_s32(dst32));
            }
        }
    }
    
    // V3-32: Unpack 3 32-bit values with W component set to default
    static void unpackV3_32(void* dst, const void* src, size_t count, uint32_t wValue = 0x3F800000) {
        prefetchData(src, count * 12);
        
        const uint32_t* srcData = static_cast<const uint32_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        uint32x4_t wVector = vdupq_n_u32(wValue);
        
        for (size_t i = 0; i < count; ++i) {
            // Load 3 elements
            uint32x2_t xy = vld1_u32(srcData + i * 3);
            uint32_t z = srcData[i * 3 + 2];
            
            // Combine into 4-element vector
            uint32x4_t result = vcombine_u32(xy, vdup_n_u32(z));
            result = vsetq_lane_u32(wValue, result, 3);
            
            vst1q_u32(dstData + i * 4, result);
        }
    }
    
    // V3-16: Unpack 3 16-bit values to 32-bit with W component
    static void unpackV3_16(void* dst, const void* src, size_t count, bool usn = false, uint32_t wValue = 0x3F800000) {
        prefetchData(src, count * 6);
        
        const uint16_t* srcData = static_cast<const uint16_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        for (size_t i = 0; i < count; ++i) {
            // Load 3 16-bit values
            uint16_t xyz[4] = {srcData[i * 3], srcData[i * 3 + 1], srcData[i * 3 + 2], 0};
            uint16x4_t src16 = vld1_u16(xyz);
            
            uint32x4_t result;
            if (usn) {
                result = vmovl_u16(src16);
            } else {
                int16x4_t srcSigned = vreinterpret_s16_u16(src16);
                result = vreinterpretq_u32_s32(vmovl_s16(srcSigned));
            }
            
            // Set W component
            result = vsetq_lane_u32(wValue, result, 3);
            vst1q_u32(dstData + i * 4, result);
        }
    }
    
    // V2-32: Unpack 2 32-bit values with ZW components set to defaults
    static void unpackV2_32(void* dst, const void* src, size_t count, uint32_t zValue = 0, uint32_t wValue = 0x3F800000) {
        prefetchData(src, count * 8);
        
        const uint32_t* srcData = static_cast<const uint32_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        for (size_t i = 0; i < count; ++i) {
            uint32x2_t xy = vld1_u32(srcData + i * 2);
            uint32x2_t zw = vcreate_u32((static_cast<uint64_t>(wValue) << 32) | zValue);
            uint32x4_t result = vcombine_u32(xy, zw);
            
            vst1q_u32(dstData + i * 4, result);
        }
    }
    
    // V1-32: Unpack 1 32-bit value with YZW components set to defaults
    static void unpackV1_32(void* dst, const void* src, size_t count, uint32_t yValue = 0, uint32_t zValue = 0, uint32_t wValue = 0x3F800000) {
        prefetchData(src, count * 4);
        
        const uint32_t* srcData = static_cast<const uint32_t*>(src);
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        uint32x4_t defaultVector = {0, yValue, zValue, wValue};
        
        for (size_t i = 0; i < count; ++i) {
            uint32x4_t result = defaultVector;
            result = vsetq_lane_u32(srcData[i], result, 0);
            
            vst1q_u32(dstData + i * 4, result);
        }
    }
    
    // Masked unpacking with NEON optimization
    static void unpackMasked(void* dst, const void* src, const uint32_t* mask, 
                           size_t count, VIFUnpackMode mode, bool usn = false) {
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        
        // Load mask into NEON register for fast comparison
        uint32x4_t maskVector = vld1q_u32(mask);
        uint32x4_t zeroVector = vdupq_n_u32(0);
        uint32x4_t maskComparison = vceqq_u32(maskVector, zeroVector);
        
        for (size_t i = 0; i < count; ++i) {
            uint32x4_t newData, oldData;
            
            // Unpack new data based on mode
            switch (mode) {
                case VIF_UNPACK_V4_32:
                    newData = vld1q_u32(static_cast<const uint32_t*>(src) + i * 4);
                    break;
                case VIF_UNPACK_V4_16: {
                    const uint16_t* src16 = static_cast<const uint16_t*>(src) + i * 4;
                    uint16x4_t temp = vld1_u16(src16);
                    newData = usn ? vmovl_u16(temp) : vreinterpretq_u32_s32(vmovl_s16(vreinterpret_s16_u16(temp)));
                    break;
                }
                default:
                    // Fallback for other modes
                    newData = vdupq_n_u32(0);
                    break;
            }
            
            // Load old data
            oldData = vld1q_u32(dstData + i * 4);
            
            // Apply mask: keep old data where mask is 0, use new data where mask is non-zero
            uint32x4_t result = vbslq_u32(maskComparison, oldData, newData);
            
            vst1q_u32(dstData + i * 4, result);
        }
    }
    
    // Optimized stencil/write mask application
    static void applyWriteMask(void* dst, const void* src, uint32_t writeMask, size_t count) {
        uint32_t* dstData = static_cast<uint32_t*>(dst);
        const uint32_t* srcData = static_cast<const uint32_t*>(src);
        
        // Create mask vectors for each component
        uint32x4_t maskVector = {
            (writeMask & 1) ? 0xFFFFFFFF : 0,
            (writeMask & 2) ? 0xFFFFFFFF : 0,
            (writeMask & 4) ? 0xFFFFFFFF : 0,
            (writeMask & 8) ? 0xFFFFFFFF : 0
        };
        
        for (size_t i = 0; i < count; ++i) {
            uint32x4_t srcVector = vld1q_u32(srcData + i * 4);
            uint32x4_t dstVector = vld1q_u32(dstData + i * 4);
            
            // Apply write mask using NEON blend
            uint32x4_t result = vbslq_u32(maskVector, srcVector, dstVector);
            
            vst1q_u32(dstData + i * 4, result);
        }
    }
};

// Public API implementation
void vifUnpackV4_32(void* dst, const void* src, size_t count, bool usn) {
    NEONVIFUnpacker::unpackV4_32(dst, src, count, usn);
}

void vifUnpackV4_16(void* dst, const void* src, size_t count, bool usn) {
    NEONVIFUnpacker::unpackV4_16(dst, src, count, usn);
}

void vifUnpackV4_8(void* dst, const void* src, size_t count, bool usn) {
    NEONVIFUnpacker::unpackV4_8(dst, src, count, usn);
}

void vifUnpackV3_32(void* dst, const void* src, size_t count, uint32_t wComponent) {
    NEONVIFUnpacker::unpackV3_32(dst, src, count, wComponent);
}

void vifUnpackV3_16(void* dst, const void* src, size_t count, bool usn, uint32_t wComponent) {
    NEONVIFUnpacker::unpackV3_16(dst, src, count, usn, wComponent);
}

void vifUnpackV2_32(void* dst, const void* src, size_t count, uint32_t zComponent, uint32_t wComponent) {
    NEONVIFUnpacker::unpackV2_32(dst, src, count, zComponent, wComponent);
}

void vifUnpackV1_32(void* dst, const void* src, size_t count, uint32_t yComponent, uint32_t zComponent, uint32_t wComponent) {
    NEONVIFUnpacker::unpackV1_32(dst, src, count, yComponent, zComponent, wComponent);
}

void vifUnpackMasked(void* dst, const void* src, const uint32_t* mask, size_t count, VIFUnpackMode mode, bool usn) {
    NEONVIFUnpacker::unpackMasked(dst, src, mask, count, mode, usn);
}

void vifApplyWriteMask(void* dst, const void* src, uint32_t writeMask, size_t count) {
    NEONVIFUnpacker::applyWriteMask(dst, src, writeMask, count);
}

// Batch processing for improved throughput
void vifUnpackBatch(const VIFUnpackBatch* batches, size_t batchCount) {
    for (size_t i = 0; i < batchCount; ++i) {
        const VIFUnpackBatch& batch = batches[i];
        
        switch (batch.mode) {
            case VIF_UNPACK_V4_32:
                vifUnpackV4_32(batch.dst, batch.src, batch.count, batch.usn);
                break;
            case VIF_UNPACK_V4_16:
                vifUnpackV4_16(batch.dst, batch.src, batch.count, batch.usn);
                break;
            case VIF_UNPACK_V4_8:
                vifUnpackV4_8(batch.dst, batch.src, batch.count, batch.usn);
                break;
            case VIF_UNPACK_V3_32:
                vifUnpackV3_32(batch.dst, batch.src, batch.count, batch.wComponent);
                break;
            case VIF_UNPACK_V3_16:
                vifUnpackV3_16(batch.dst, batch.src, batch.count, batch.usn, batch.wComponent);
                break;
            case VIF_UNPACK_V2_32:
                vifUnpackV2_32(batch.dst, batch.src, batch.count, batch.zComponent, batch.wComponent);
                break;
            case VIF_UNPACK_V1_32:
                vifUnpackV1_32(batch.dst, batch.src, batch.count, batch.yComponent, batch.zComponent, batch.wComponent);
                break;
        }
        
        // Apply write mask if specified
        if (batch.writeMask != 0xF) {
            vifApplyWriteMask(batch.dst, batch.dst, batch.writeMask, batch.count);
        }
    }
}

} // namespace ARM64Recompiler
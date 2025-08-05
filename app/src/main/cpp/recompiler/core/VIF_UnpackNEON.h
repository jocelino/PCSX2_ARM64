// ARM64 NEON Optimized VIF Unpacking Header
// High-performance data unpacking interface

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

// VIF unpacking modes
enum VIFUnpackMode {
    VIF_UNPACK_V1_32,  // 1 x 32-bit
    VIF_UNPACK_V2_32,  // 2 x 32-bit
    VIF_UNPACK_V3_32,  // 3 x 32-bit
    VIF_UNPACK_V4_32,  // 4 x 32-bit
    VIF_UNPACK_V1_16,  // 1 x 16-bit
    VIF_UNPACK_V2_16,  // 2 x 16-bit
    VIF_UNPACK_V3_16,  // 3 x 16-bit
    VIF_UNPACK_V4_16,  // 4 x 16-bit
    VIF_UNPACK_V1_8,   // 1 x 8-bit
    VIF_UNPACK_V2_8,   // 2 x 8-bit
    VIF_UNPACK_V3_8,   // 3 x 8-bit
    VIF_UNPACK_V4_8,   // 4 x 8-bit
    VIF_UNPACK_V1_5,   // 1 x 5-bit (special)
    VIF_UNPACK_V2_5,   // 2 x 5-bit (special)
    VIF_UNPACK_V3_5,   // 3 x 5-bit (special)
    VIF_UNPACK_V4_5    // 4 x 5-bit (special)
};

// Batch processing structure for better performance
struct VIFUnpackBatch {
    void* dst;              // Destination buffer (16-byte aligned)
    const void* src;        // Source data
    size_t count;           // Number of elements to unpack
    VIFUnpackMode mode;     // Unpacking mode
    bool usn;               // Unsigned/signed flag
    uint32_t writeMask;     // Write mask (0xF = write all components)
    uint32_t yComponent;    // Default Y component value
    uint32_t zComponent;    // Default Z component value
    uint32_t wComponent;    // Default W component value
};

// High-level VIF unpacking functions with NEON optimization
void vifUnpackV4_32(void* dst, const void* src, size_t count, bool usn = false);
void vifUnpackV4_16(void* dst, const void* src, size_t count, bool usn = false);
void vifUnpackV4_8(void* dst, const void* src, size_t count, bool usn = false);

void vifUnpackV3_32(void* dst, const void* src, size_t count, uint32_t wComponent = 0x3F800000);
void vifUnpackV3_16(void* dst, const void* src, size_t count, bool usn = false, uint32_t wComponent = 0x3F800000);
void vifUnpackV3_8(void* dst, const void* src, size_t count, bool usn = false, uint32_t wComponent = 0x3F800000);

void vifUnpackV2_32(void* dst, const void* src, size_t count, uint32_t zComponent = 0, uint32_t wComponent = 0x3F800000);
void vifUnpackV2_16(void* dst, const void* src, size_t count, bool usn = false, uint32_t zComponent = 0, uint32_t wComponent = 0x3F800000);
void vifUnpackV2_8(void* dst, const void* src, size_t count, bool usn = false, uint32_t zComponent = 0, uint32_t wComponent = 0x3F800000);

void vifUnpackV1_32(void* dst, const void* src, size_t count, uint32_t yComponent = 0, uint32_t zComponent = 0, uint32_t wComponent = 0x3F800000);
void vifUnpackV1_16(void* dst, const void* src, size_t count, bool usn = false, uint32_t yComponent = 0, uint32_t zComponent = 0, uint32_t wComponent = 0x3F800000);
void vifUnpackV1_8(void* dst, const void* src, size_t count, bool usn = false, uint32_t yComponent = 0, uint32_t zComponent = 0, uint32_t wComponent = 0x3F800000);

// Masked unpacking with conditional writes
void vifUnpackMasked(void* dst, const void* src, const uint32_t* mask, size_t count, VIFUnpackMode mode, bool usn = false);

// Write mask application
void vifApplyWriteMask(void* dst, const void* src, uint32_t writeMask, size_t count);

// Batch processing for multiple unpack operations
void vifUnpackBatch(const VIFUnpackBatch* batches, size_t batchCount);

// Performance measurement and statistics
struct VIFStats {
    uint64_t totalUnpackOperations;
    uint64_t totalElementsUnpacked;
    uint64_t neonOptimizedOperations;
    uint64_t fallbackOperations;
    double averageUnpackTime;
    double neonSpeedup;
};

VIFStats getVIFUnpackStats();
void resetVIFUnpackStats();

// NEON capability detection
bool isNEONAvailable();
bool isCryptoExtensionAvailable();

// Utility functions for alignment and validation
inline bool isAligned16(const void* ptr) {
    return (reinterpret_cast<uintptr_t>(ptr) & 15) == 0;
}

inline bool isValidUnpackMode(VIFUnpackMode mode) {
    return mode >= VIF_UNPACK_V1_32 && mode <= VIF_UNPACK_V4_5;
}

inline size_t getElementSize(VIFUnpackMode mode) {
    switch (mode) {
        case VIF_UNPACK_V1_32: case VIF_UNPACK_V2_32: case VIF_UNPACK_V3_32: case VIF_UNPACK_V4_32:
            return 4;
        case VIF_UNPACK_V1_16: case VIF_UNPACK_V2_16: case VIF_UNPACK_V3_16: case VIF_UNPACK_V4_16:
            return 2;
        case VIF_UNPACK_V1_8: case VIF_UNPACK_V2_8: case VIF_UNPACK_V3_8: case VIF_UNPACK_V4_8:
            return 1;
        case VIF_UNPACK_V1_5: case VIF_UNPACK_V2_5: case VIF_UNPACK_V3_5: case VIF_UNPACK_V4_5:
            return 1; // Special handling required
        default:
            return 0;
    }
}

inline size_t getComponentCount(VIFUnpackMode mode) {
    switch (mode) {
        case VIF_UNPACK_V1_32: case VIF_UNPACK_V1_16: case VIF_UNPACK_V1_8: case VIF_UNPACK_V1_5:
            return 1;
        case VIF_UNPACK_V2_32: case VIF_UNPACK_V2_16: case VIF_UNPACK_V2_8: case VIF_UNPACK_V2_5:
            return 2;
        case VIF_UNPACK_V3_32: case VIF_UNPACK_V3_16: case VIF_UNPACK_V3_8: case VIF_UNPACK_V3_5:
            return 3;
        case VIF_UNPACK_V4_32: case VIF_UNPACK_V4_16: case VIF_UNPACK_V4_8: case VIF_UNPACK_V4_5:
            return 4;
        default:
            return 0;
    }
}

} // namespace ARM64Recompiler
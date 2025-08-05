// ARM64 NEON Intrinsics Header
// High-performance SIMD operations interface

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {
namespace NEONIntrinsics {

// Matrix and vector operations
void matrixMultiply4x4(float* result, const float* a, const float* b);
void transformVertices(float* vertices, const float* matrix, size_t vertexCount);

// Color format conversions
void convertRGBA8ToFloat(float* dst, const uint8_t* src, size_t pixelCount);
void convertFloatToRGBA8(uint8_t* dst, const float* src, size_t pixelCount);

// Texture filtering
void bilinearFilter(float* dst, const float* src, int srcWidth, int srcHeight, float u, float v);

// Audio processing
void mixAudioChannels(float* output, const float* input1, const float* input2, 
                     float volume1, float volume2, size_t sampleCount);
void applyAudioFilter(float* samples, const float* coefficients, size_t sampleCount, size_t filterOrder);

// Memory operations
void fastMemoryCopy(void* dst, const void* src, size_t size);
void fastMemorySet(void* dst, uint8_t value, size_t size);

// Performance measurement
uint64_t getCycleCount();
double getElapsedSeconds(uint64_t startCycles, uint64_t endCycles);

} // namespace NEONIntrinsics
} // namespace ARM64Recompiler
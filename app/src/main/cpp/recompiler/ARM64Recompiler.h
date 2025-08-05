// ARM64 Recompiler Main Header
// Optimized recompiler library for PCSX2 ARM64

#pragma once

#include <cstdint>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// Memory allocation functions
void* allocateMicroProgram();
void releaseMicroProgram(void* ptr);
void* allocateCodeBlock();
void releaseCodeBlock(void* ptr);
void clearMemoryFast(void* ptr, size_t size);

// VIF unpacking functions
void vifUnpackV4_32_simple(void* dst, const void* src, size_t count);
void vifUnpackV4_16_simple(void* dst, const void* src, size_t count);

// Vector math functions
void vectorAdd4f_simple(float* result, const float* a, const float* b);
void vectorMul4f_simple(float* result, const float* a, const float* b);
float vectorDot3f_simple(const float* a, const float* b);

// Memory operations
void fastMemcpyNEON(void* dst, const void* src, size_t size);
void fastMemsetNEON(void* ptr, uint8_t value, size_t size);

// Matrix operations
void matrixMul4x4NEON(float* result, const float* a, const float* b);

// Color conversion
void convertRGBA8ToFloat_NEON(float* dst, const uint8_t* src, size_t pixelCount);

// Performance and feature detection
uint64_t getPerformanceCounter();
int isNEONAvailable();
uint32_t getARM64Features();

#ifdef __cplusplus
}
#endif
// ARM64 Vector Operations Header
// Optimized vector operations using NEON

#pragma once

namespace ARM64Recompiler {

// Vector math operations
void vectorAdd4f(float* result, const float* a, const float* b);
void vectorSub4f(float* result, const float* a, const float* b);
void vectorMul4f(float* result, const float* a, const float* b);
void vectorMadd4f(float* result, const float* a, const float* b, const float* c);

float vectorDot3f(const float* a, const float* b);
void vectorNormalize3f(float* result, const float* input);

} // namespace ARM64Recompiler
// ARM64 NEON Vector Math Header
// Advanced vector mathematics using NEON instructions

#pragma once

namespace ARM64Recompiler {

// Matrix operations
void matrixMultiply4x4NEON(float* result, const float* a, const float* b);

// Vector operations
void vectorCrossProduct(float* result, const float* a, const float* b);
float vectorLength(const float* vector);
void vectorNormalize(float* result, const float* vector);

// Quaternion operations
void quaternionMultiply(float* result, const float* q1, const float* q2);

// Transformation operations
void transformPoint(float* result, const float* point, const float* matrix);

} // namespace ARM64Recompiler
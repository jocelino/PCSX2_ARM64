// ARM64 Code Generation Header
// High-level code generation utilities

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

enum VectorOp {
    VECTOR_ADD,
    VECTOR_SUB,
    VECTOR_MUL,
    VECTOR_MADD
};

// Code generation functions
void generatePrologue(uint8_t* code, size_t& offset);
void generateEpilogue(uint8_t* code, size_t& offset);
void generateVectorOperation(uint8_t* code, size_t& offset, VectorOp op, 
                           uint32_t dst, uint32_t src1, uint32_t src2);

} // namespace ARM64Recompiler
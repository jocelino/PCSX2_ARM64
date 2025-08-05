// ARM64 VU Compiler Header
// Compiles VU microcode to optimized ARM64 assembly

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

// VU compilation functions
void* compileVUMicrocode(const uint32_t* microcode, size_t instructionCount);

} // namespace ARM64Recompiler
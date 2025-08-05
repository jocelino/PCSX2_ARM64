// ARM64 R5900 Recompiler Header
// PlayStation 2 main CPU recompiler optimized for ARM64

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

// R5900 compilation functions
void* compileR5900CodeBlock(const uint32_t* instructions, size_t count);

} // namespace ARM64Recompiler
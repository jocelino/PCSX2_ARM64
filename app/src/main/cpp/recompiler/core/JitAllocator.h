// ARM64 JIT Allocator Header
// Allocates executable memory for compiled code

#pragma once

#include <cstddef>

namespace ARM64Recompiler {

// Executable memory allocation functions
void* allocateExecutableMemory(size_t size);
void deallocateExecutableMemory(void* ptr, size_t size);

} // namespace ARM64Recompiler
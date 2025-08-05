// ARM64 GS Memory Operations Header
// Optimized memory operations for Graphics Synthesizer

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

// GS memory operation functions
void gsMemoryTransfer(void* dst, const void* src, size_t size);
void gsMemorySet(void* ptr, uint8_t value, size_t size);
void gsSwizzleTexture(void* dst, const void* src, uint32_t width, uint32_t height, uint32_t bpp);
void gsUnswizzleTexture(void* dst, const void* src, uint32_t width, uint32_t height, uint32_t bpp);

} // namespace ARM64Recompiler
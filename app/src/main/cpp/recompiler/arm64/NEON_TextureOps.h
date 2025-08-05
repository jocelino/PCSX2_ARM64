// ARM64 NEON Texture Operations Header
// Optimized texture processing using NEON instructions

#pragma once

#include <cstdint>

namespace ARM64Recompiler {

// Color format conversion
void convertRGBA32ToRGBA16(uint16_t* dst, const uint32_t* src, size_t pixelCount);

// Texture filtering
void applyBilinearFilter(uint8_t* dst, const uint8_t* src, int width, int height, float u, float v);

// Texture swizzling (PlayStation 2 specific)
void swizzleTexture(uint8_t* dst, const uint8_t* src, int width, int height, int bpp);
void unswizzleTexture(uint8_t* dst, const uint8_t* src, int width, int height, int bpp);

// Mipmap generation
void generateMipmaps(uint8_t* mipmaps, const uint8_t* baseTexture, int width, int height, int levels);

} // namespace ARM64Recompiler
// ARM64 GS Texture Cache Header
// Optimized texture cache operations for Graphics Synthesizer

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

struct TextureEntry;

// Texture cache functions
uint32_t calculateTextureHash(const void* data, size_t size);
TextureEntry* lookupTexture(uint32_t hash);
void cacheTexture(uint32_t hash, void* data, uint32_t width, uint32_t height, uint32_t format);
void invalidateTextureCache(uint32_t hash);
void clearTextureCache();

} // namespace ARM64Recompiler
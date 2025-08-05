// ARM64 GS Texture Cache Implementation
// Optimized texture cache operations for Graphics Synthesizer

#include "GS_TextureCache.h"
#include <arm_neon.h>
#include <unordered_map>
#include <mutex>

namespace ARM64Recompiler {

struct TextureEntry {
    uint32_t hash;
    void* data;
    uint32_t width;
    uint32_t height;
    uint32_t format;
};

class GSTextureCache {
private:
    std::unordered_map<uint32_t, TextureEntry> cache;
    std::mutex cacheMutex;
    
public:
    TextureEntry* getTexture(uint32_t hash) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        auto it = cache.find(hash);
        return (it != cache.end()) ? &it->second : nullptr;
    }
    
    void addTexture(uint32_t hash, const TextureEntry& entry) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        cache[hash] = entry;
    }
    
    void invalidateTexture(uint32_t hash) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        cache.erase(hash);
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(cacheMutex);
        cache.clear();
    }
};

static GSTextureCache globalTextureCache;

uint32_t calculateTextureHash(const void* data, size_t size) {
    // Simple hash calculation using NEON
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t hash = 0;
    
    size_t neonSize = size & ~15;
    for (size_t i = 0; i < neonSize; i += 16) {
        uint8x16_t chunk = vld1q_u8(bytes + i);
        uint32x4_t words = vreinterpretq_u32_u8(chunk);
        
        for (int j = 0; j < 4; j++) {
            hash ^= vgetq_lane_u32(words, j);
            hash = hash * 1103515245 + 12345;
        }
    }
    
    // Handle remaining bytes
    for (size_t i = neonSize; i < size; i++) {
        hash ^= bytes[i];
        hash = hash * 1103515245 + 12345;
    }
    
    return hash;
}

TextureEntry* lookupTexture(uint32_t hash) {
    return globalTextureCache.getTexture(hash);
}

void cacheTexture(uint32_t hash, void* data, uint32_t width, uint32_t height, uint32_t format) {
    TextureEntry entry;
    entry.hash = hash;
    entry.data = data;
    entry.width = width;
    entry.height = height;
    entry.format = format;
    
    globalTextureCache.addTexture(hash, entry);
}

void invalidateTextureCache(uint32_t hash) {
    globalTextureCache.invalidateTexture(hash);
}

void clearTextureCache() {
    globalTextureCache.clear();
}

} // namespace ARM64Recompiler
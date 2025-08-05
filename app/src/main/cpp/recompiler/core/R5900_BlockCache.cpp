// ARM64 R5900 Block Cache Implementation
// Caches compiled R5900 instruction blocks

#include "R5900_BlockCache.h"
#include "MemoryPool.h"
#include <unordered_map>
#include <mutex>

namespace ARM64Recompiler {

class R5900BlockCache {
private:
    std::unordered_map<uint32_t, void*> cachedBlocks;
    std::mutex cacheMutex;
    
public:
    void* getCachedBlock(uint32_t pc) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        auto it = cachedBlocks.find(pc);
        return (it != cachedBlocks.end()) ? it->second : nullptr;
    }
    
    void cacheBlock(uint32_t pc, void* block) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        cachedBlocks[pc] = block;
    }
    
    void invalidateBlock(uint32_t pc) {
        std::lock_guard<std::mutex> lock(cacheMutex);
        auto it = cachedBlocks.find(pc);
        if (it != cachedBlocks.end()) {
            releaseLargeBuffer(it->second);
            cachedBlocks.erase(it);
        }
    }
    
    void clearCache() {
        std::lock_guard<std::mutex> lock(cacheMutex);
        for (auto& pair : cachedBlocks) {
            releaseLargeBuffer(pair.second);
        }
        cachedBlocks.clear();
    }
    
    size_t getCacheSize() const {
        std::lock_guard<std::mutex> lock(cacheMutex);
        return cachedBlocks.size();
    }
};

static R5900BlockCache globalBlockCache;

void* getR5900CachedBlock(uint32_t pc) {
    return globalBlockCache.getCachedBlock(pc);
}

void cacheR5900Block(uint32_t pc, void* block) {
    globalBlockCache.cacheBlock(pc, block);
}

void invalidateR5900Block(uint32_t pc) {
    globalBlockCache.invalidateBlock(pc);
}

void clearR5900Cache() {
    globalBlockCache.clearCache();
}

size_t getR5900CacheSize() {
    return globalBlockCache.getCacheSize();
}

} // namespace ARM64Recompiler
// ARM64 Block Manager Implementation
// Manages compiled code blocks and cache invalidation

#include "BlockManager.h"
#include "MemoryPool.h"
#include <mutex>
#include <unordered_map>

namespace ARM64Recompiler {

class BlockManager {
private:
    std::unordered_map<uint32_t, void*> compiledBlocks;
    std::mutex blockMutex;
    
public:
    void* getBlock(uint32_t pc) {
        std::lock_guard<std::mutex> lock(blockMutex);
        auto it = compiledBlocks.find(pc);
        return (it != compiledBlocks.end()) ? it->second : nullptr;
    }
    
    void addBlock(uint32_t pc, void* block) {
        std::lock_guard<std::mutex> lock(blockMutex);
        compiledBlocks[pc] = block;
    }
    
    void invalidateBlock(uint32_t pc) {
        std::lock_guard<std::mutex> lock(blockMutex);
        auto it = compiledBlocks.find(pc);
        if (it != compiledBlocks.end()) {
            releaseLargeBuffer(it->second);
            compiledBlocks.erase(it);
        }
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(blockMutex);
        for (auto& pair : compiledBlocks) {
            releaseLargeBuffer(pair.second);
        }
        compiledBlocks.clear();
    }
};

static BlockManager globalBlockManager;

void* getCompiledBlock(uint32_t pc) {
    return globalBlockManager.getBlock(pc);
}

void addCompiledBlock(uint32_t pc, void* block) {
    globalBlockManager.addBlock(pc, block);
}

void invalidateCompiledBlock(uint32_t pc) {
    globalBlockManager.invalidateBlock(pc);
}

void clearAllBlocks() {
    globalBlockManager.clear();
}

} // namespace ARM64Recompiler
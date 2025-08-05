// ARM64 Optimized Memory Pool Implementation
// High-performance aligned memory allocation for PCSX2 recompiler

#include "MemoryPool.h"
#include <arm_neon.h>
#include <atomic>
#include <mutex>
#include <sys/mman.h>
#include <unistd.h>

namespace ARM64Recompiler {

// ARM64-optimized memory pool with 64-byte cache line alignment
template<size_t BlockSize, size_t Alignment>
class OptimizedMemoryPool {
private:
    struct alignas(64) FreeBlock {
        FreeBlock* next;
        char padding[64 - sizeof(FreeBlock*)];
    };
    
    std::atomic<FreeBlock*> freeList{nullptr};
    std::vector<void*> allocatedChunks;
    std::mutex chunkMutex;
    size_t chunkSize;
    size_t blocksPerChunk;
    
    // ARM64 cache line prefetch optimization
    void prefetchBlock(void* ptr) const {
        __builtin_prefetch(ptr, 1, 3); // Write prefetch, high temporal locality
        __builtin_prefetch(static_cast<char*>(ptr) + 64, 1, 3);
    }
    
    void* allocateChunk() {
        size_t totalSize = chunkSize * BlockSize;
        void* chunk = mmap(nullptr, totalSize, 
                          PROT_READ | PROT_WRITE, 
                          MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        
        if (chunk == MAP_FAILED) {
            return nullptr;
        }
        
        // ARM64 memory barrier for consistency
        __builtin_arm_dmb(0xF); // Full system barrier
        
        // Initialize free list for this chunk
        char* block = static_cast<char*>(chunk);
        FreeBlock* prevBlock = nullptr;
        
        for (size_t i = 0; i < blocksPerChunk; ++i) {
            FreeBlock* currentBlock = reinterpret_cast<FreeBlock*>(block);
            if (prevBlock) {
                prevBlock->next = currentBlock;
            }
            currentBlock->next = nullptr;
            prevBlock = currentBlock;
            block += BlockSize;
        }
        
        // Link to main free list atomically
        FreeBlock* firstBlock = reinterpret_cast<FreeBlock*>(chunk);
        FreeBlock* lastBlock = prevBlock;
        
        FreeBlock* oldHead = freeList.load(std::memory_order_acquire);
        do {
            lastBlock->next = oldHead;
        } while (!freeList.compare_exchange_weak(oldHead, firstBlock, 
                                                std::memory_order_release, 
                                                std::memory_order_relaxed));
        
        return chunk;
    }
    
public:
    OptimizedMemoryPool(size_t initialChunks = 4) 
        : chunkSize(std::max(size_t(1024), (4096 / BlockSize) * 4))
        , blocksPerChunk(chunkSize) {
        
        // Pre-allocate initial chunks for better performance
        for (size_t i = 0; i < initialChunks; ++i) {
            void* chunk = allocateChunk();
            if (chunk) {
                std::lock_guard<std::mutex> lock(chunkMutex);
                allocatedChunks.push_back(chunk);
            }
        }
    }
    
    ~OptimizedMemoryPool() {
        std::lock_guard<std::mutex> lock(chunkMutex);
        for (void* chunk : allocatedChunks) {
            munmap(chunk, chunkSize * BlockSize);
        }
    }
    
    void* acquire() {
        FreeBlock* block = freeList.load(std::memory_order_acquire);
        
        while (block) {
            FreeBlock* next = block->next;
            if (freeList.compare_exchange_weak(block, next, 
                                              std::memory_order_release, 
                                              std::memory_order_relaxed)) {
                prefetchBlock(block);
                return block;
            }
        }
        
        // No free blocks, allocate new chunk
        void* newChunk = allocateChunk();
        if (newChunk) {
            std::lock_guard<std::mutex> lock(chunkMutex);
            allocatedChunks.push_back(newChunk);
            
            // Try again now that we have more blocks
            return acquire();
        }
        
        return nullptr;
    }
    
    void release(void* ptr) {
        if (!ptr) return;
        
        FreeBlock* block = static_cast<FreeBlock*>(ptr);
        
        // ARM64 cache line flush for released memory
        __builtin_arm_dmb(0xE); // Store barrier
        
        FreeBlock* oldHead = freeList.load(std::memory_order_relaxed);
        do {
            block->next = oldHead;
        } while (!freeList.compare_exchange_weak(oldHead, block, 
                                                std::memory_order_release, 
                                                std::memory_order_relaxed));
    }
    
    size_t getMemoryUsage() const {
        std::lock_guard<std::mutex> lock(chunkMutex);
        return allocatedChunks.size() * chunkSize * BlockSize;
    }
};

// Specialized pools for different PCSX2 components
static OptimizedMemoryPool<256, 64> microProgramPool(8);    // MicroVU programs
static OptimizedMemoryPool<128, 32> blockCachePool(16);     // Code blocks
static OptimizedMemoryPool<64, 16> instructionPool(32);     // Instructions
static OptimizedMemoryPool<4096, 64> largeBufferPool(4);    // Large allocations

// Global allocation functions optimized for ARM64
void* allocateMicroProgram() {
    return microProgramPool.acquire();
}

void releaseMicroProgram(void* ptr) {
    microProgramPool.release(ptr);
}

void* allocateCodeBlock() {
    return blockCachePool.acquire();
}

void releaseCodeBlock(void* ptr) {
    blockCachePool.release(ptr);
}

void* allocateInstruction() {
    return instructionPool.acquire();
}

void releaseInstruction(void* ptr) {
    instructionPool.release(ptr);
}

void* allocateLargeBuffer() {
    return largeBufferPool.acquire();
}

void releaseLargeBuffer(void* ptr) {
    largeBufferPool.release(ptr);
}

// ARM64-specific memory clearing with NEON optimization
void fastMemoryClearing(void* ptr, size_t size) {
    if (size < 64) {
        // Small sizes - use regular memset
        std::memset(ptr, 0, size);
        return;
    }
    
    // Use NEON for large memory clearing
    uint8_t* data = static_cast<uint8_t*>(ptr);
    const uint8x16_t zero = vdupq_n_u8(0);
    
    // Clear 64-byte chunks with NEON
    size_t chunks = size / 64;
    for (size_t i = 0; i < chunks; ++i) {
        vst1q_u8(data + 0, zero);
        vst1q_u8(data + 16, zero);
        vst1q_u8(data + 32, zero);
        vst1q_u8(data + 48, zero);
        data += 64;
    }
    
    // Handle remaining bytes
    size_t remaining = size % 64;
    if (remaining > 0) {
        std::memset(data, 0, remaining);
    }
}

// Memory usage statistics
MemoryStats getMemoryStats() {
    MemoryStats stats;
    stats.microProgramMemory = microProgramPool.getMemoryUsage();
    stats.blockCacheMemory = blockCachePool.getMemoryUsage();
    stats.instructionMemory = instructionPool.getMemoryUsage();
    stats.largeBufferMemory = largeBufferPool.getMemoryUsage();
    stats.totalMemory = stats.microProgramMemory + stats.blockCacheMemory + 
                       stats.instructionMemory + stats.largeBufferMemory;
    return stats;
}

} // namespace ARM64Recompiler
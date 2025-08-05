// ARM64 Optimized Instruction Cache Implementation
// High-performance instruction caching with NEON optimizations

#include "InstructionCache.h"
#include <arm_neon.h>
#include <atomic>
#include <unordered_map>
#include <shared_mutex>

namespace ARM64Recompiler {

// ARM64-optimized hash function using CRC32 instruction
constexpr uint32_t ARM64_CRC_SEED = 0xFFFFFFFF;

inline uint32_t fastHash32(uint32_t value) {
    // Use ARM64 CRC32 instruction if available
    #ifdef __ARM_FEATURE_CRC32
    return __builtin_arm_crc32w(ARM64_CRC_SEED, value);
    #else
    // Fallback hash
    value ^= value >> 16;
    value *= 0x85ebca6b;
    value ^= value >> 13;
    value *= 0xc2b2ae35;
    value ^= value >> 16;
    return value;
    #endif
}

inline uint64_t fastHash64(uint64_t value) {
    #ifdef __ARM_FEATURE_CRC32
    uint32_t low = static_cast<uint32_t>(value);
    uint32_t high = static_cast<uint32_t>(value >> 32);
    return (static_cast<uint64_t>(__builtin_arm_crc32w(ARM64_CRC_SEED, high)) << 32) |
           __builtin_arm_crc32w(ARM64_CRC_SEED, low);
    #else
    value ^= value >> 33;
    value *= 0xff51afd7ed558ccdULL;
    value ^= value >> 33;
    value *= 0xc4ceb9fe1a85ec53ULL;
    value ^= value >> 33;
    return value;
    #endif
}

// Instruction cache entry with ARM64 optimizations
struct alignas(64) CacheEntry {
    uint32_t pc;                    // Program counter
    uint32_t hash;                  // Instruction hash
    uint64_t compiledCode;          // Pointer to compiled ARM64 code
    uint32_t codeSize;              // Size of compiled code
    uint32_t hitCount;              // Usage frequency
    std::atomic<uint64_t> lastUsed; // Last access timestamp
    uint8_t padding[24];            // Cache line padding
    
    CacheEntry() : pc(0), hash(0), compiledCode(0), codeSize(0), hitCount(0), lastUsed(0) {}
};

// High-performance cache with ARM64 optimizations
class OptimizedInstructionCache {
private:
    static constexpr size_t CACHE_SIZE = 32768;        // 32K entries
    static constexpr size_t CACHE_MASK = CACHE_SIZE - 1;
    static constexpr size_t CACHE_WAYS = 4;            // 4-way associative
    static constexpr uint64_t EVICTION_THRESHOLD = 1000000; // LRU threshold
    
    alignas(64) CacheEntry cache[CACHE_SIZE];
    std::shared_mutex cacheMutex;
    std::atomic<uint64_t> globalTimer{0};
    std::atomic<uint64_t> hitCount{0};
    std::atomic<uint64_t> missCount{0};
    
    // ARM64 cache line prefetching
    void prefetchCacheLine(const CacheEntry* entry) const {
        __builtin_prefetch(entry, 0, 3);     // Read prefetch, high locality
        __builtin_prefetch(entry + 1, 0, 2); // Next entry, medium locality
    }
    
    // Find cache entry using optimized search
    CacheEntry* findEntry(uint32_t pc, uint32_t hash) {
        size_t baseIndex = hash & CACHE_MASK;
        
        // Check all ways in the set
        for (size_t way = 0; way < CACHE_WAYS; ++way) {
            size_t index = (baseIndex + way) & CACHE_MASK;
            CacheEntry* entry = &cache[index];
            
            prefetchCacheLine(entry);
            
            if (entry->pc == pc && entry->hash == hash) {
                // Update access statistics
                entry->hitCount++;
                entry->lastUsed.store(globalTimer.fetch_add(1), std::memory_order_relaxed);
                return entry;
            }
        }
        
        return nullptr;
    }
    
    // LRU eviction with ARM64 optimizations
    CacheEntry* findEvictionCandidate(size_t baseIndex) {
        CacheEntry* oldest = nullptr;
        uint64_t oldestTime = UINT64_MAX;
        
        for (size_t way = 0; way < CACHE_WAYS; ++way) {
            size_t index = (baseIndex + way) & CACHE_MASK;
            CacheEntry* entry = &cache[index];
            
            uint64_t lastUsed = entry->lastUsed.load(std::memory_order_relaxed);
            if (lastUsed < oldestTime) {
                oldestTime = lastUsed;
                oldest = entry;
            }
        }
        
        return oldest;
    }
    
public:
    OptimizedInstructionCache() {
        // Initialize cache with NEON if possible
        const uint8x16_t zero = vdupq_n_u8(0);
        uint8_t* cacheBytes = reinterpret_cast<uint8_t*>(cache);
        
        size_t totalSize = sizeof(cache);
        size_t chunks = totalSize / 16;
        
        for (size_t i = 0; i < chunks; ++i) {
            vst1q_u8(cacheBytes + i * 16, zero);
        }
    }
    
    // Lookup compiled code for PC
    CompiledCode* lookup(uint32_t pc, const uint32_t* instructions, size_t instrCount) {
        // Generate hash from instruction sequence
        uint32_t hash = fastHash32(pc);
        for (size_t i = 0; i < instrCount; ++i) {
            hash = fastHash32(hash ^ instructions[i]);
        }
        
        // Fast path: read lock only
        {
            std::shared_lock<std::shared_mutex> lock(cacheMutex);
            CacheEntry* entry = findEntry(pc, hash);
            if (entry) {
                hitCount.fetch_add(1, std::memory_order_relaxed);
                return reinterpret_cast<CompiledCode*>(entry->compiledCode);
            }
        }
        
        missCount.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    
    // Insert compiled code into cache
    void insert(uint32_t pc, const uint32_t* instructions, size_t instrCount, 
                CompiledCode* code, uint32_t codeSize) {
        uint32_t hash = fastHash32(pc);
        for (size_t i = 0; i < instrCount; ++i) {
            hash = fastHash32(hash ^ instructions[i]);
        }
        
        size_t baseIndex = hash & CACHE_MASK;
        
        std::unique_lock<std::shared_mutex> lock(cacheMutex);
        
        // Try to find empty slot first
        for (size_t way = 0; way < CACHE_WAYS; ++way) {
            size_t index = (baseIndex + way) & CACHE_MASK;
            CacheEntry* entry = &cache[index];
            
            if (entry->compiledCode == 0) {
                entry->pc = pc;
                entry->hash = hash;
                entry->compiledCode = reinterpret_cast<uint64_t>(code);
                entry->codeSize = codeSize;
                entry->hitCount = 0;
                entry->lastUsed.store(globalTimer.fetch_add(1), std::memory_order_relaxed);
                return;
            }
        }
        
        // No empty slot, evict LRU entry
        CacheEntry* victim = findEvictionCandidate(baseIndex);
        if (victim) {
            // Free old compiled code if needed
            if (victim->compiledCode) {
                // Code cleanup would go here
            }
            
            victim->pc = pc;
            victim->hash = hash;
            victim->compiledCode = reinterpret_cast<uint64_t>(code);
            victim->codeSize = codeSize;
            victim->hitCount = 0;
            victim->lastUsed.store(globalTimer.fetch_add(1), std::memory_order_relaxed);
        }
    }
    
    // Invalidate cache entries for a PC range
    void invalidateRange(uint32_t startPC, uint32_t endPC) {
        std::unique_lock<std::shared_mutex> lock(cacheMutex);
        
        for (size_t i = 0; i < CACHE_SIZE; ++i) {
            CacheEntry* entry = &cache[i];
            if (entry->pc >= startPC && entry->pc <= endPC) {
                if (entry->compiledCode) {
                    // Code cleanup would go here
                }
                entry->compiledCode = 0;
                entry->pc = 0;
                entry->hash = 0;
            }
        }
    }
    
    // Get cache statistics
    CacheStats getStats() const {
        CacheStats stats;
        stats.hitCount = hitCount.load(std::memory_order_relaxed);
        stats.missCount = missCount.load(std::memory_order_relaxed);
        stats.totalLookups = stats.hitCount + stats.missCount;
        stats.hitRate = stats.totalLookups > 0 ? 
            static_cast<double>(stats.hitCount) / stats.totalLookups : 0.0;
        
        // Count active entries
        std::shared_lock<std::shared_mutex> lock(cacheMutex);
        stats.activeEntries = 0;
        for (size_t i = 0; i < CACHE_SIZE; ++i) {
            if (cache[i].compiledCode != 0) {
                stats.activeEntries++;
            }
        }
        
        return stats;
    }
    
    // Clear entire cache
    void clear() {
        std::unique_lock<std::shared_mutex> lock(cacheMutex);
        
        for (size_t i = 0; i < CACHE_SIZE; ++i) {
            if (cache[i].compiledCode) {
                // Code cleanup would go here
            }
            cache[i] = CacheEntry{};
        }
        
        hitCount.store(0, std::memory_order_relaxed);
        missCount.store(0, std::memory_order_relaxed);
    }
};

// Global instruction cache instance
static OptimizedInstructionCache globalCache;

// Public API functions
CompiledCode* lookupInstruction(uint32_t pc, const uint32_t* instructions, size_t count) {
    return globalCache.lookup(pc, instructions, count);
}

void insertCompiledCode(uint32_t pc, const uint32_t* instructions, size_t count, 
                       CompiledCode* code, uint32_t codeSize) {
    globalCache.insert(pc, instructions, count, code, codeSize);
}

void invalidateInstructionRange(uint32_t startPC, uint32_t endPC) {
    globalCache.invalidateRange(startPC, endPC);
}

CacheStats getInstructionCacheStats() {
    return globalCache.getStats();
}

void clearInstructionCache() {
    globalCache.clear();
}

} // namespace ARM64Recompiler
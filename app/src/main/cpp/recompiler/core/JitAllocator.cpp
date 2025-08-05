// ARM64 JIT Allocator Implementation
// Allocates executable memory for compiled code

#include "JitAllocator.h"
#include <sys/mman.h>
#include <unistd.h>
#include <mutex>
#include <vector>

namespace ARM64Recompiler {

class JitAllocator {
private:
    std::vector<void*> allocatedPages;
    std::mutex allocatorMutex;
    static constexpr size_t PAGE_SIZE = 4096;
    static constexpr size_t ALLOCATION_SIZE = PAGE_SIZE * 256; // 1MB chunks
    
public:
    void* allocateExecutable(size_t size) {
        std::lock_guard<std::mutex> lock(allocatorMutex);
        
        size_t alignedSize = (size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
        
        void* memory = mmap(nullptr, alignedSize, 
                           PROT_READ | PROT_WRITE | PROT_EXEC,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        
        if (memory == MAP_FAILED) {
            return nullptr;
        }
        
        allocatedPages.push_back(memory);
        return memory;
    }
    
    void deallocateExecutable(void* ptr, size_t size) {
        std::lock_guard<std::mutex> lock(allocatorMutex);
        
        size_t alignedSize = (size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
        munmap(ptr, alignedSize);
        
        auto it = std::find(allocatedPages.begin(), allocatedPages.end(), ptr);
        if (it != allocatedPages.end()) {
            allocatedPages.erase(it);
        }
    }
    
    ~JitAllocator() {
        std::lock_guard<std::mutex> lock(allocatorMutex);
        for (void* page : allocatedPages) {
            munmap(page, ALLOCATION_SIZE);
        }
    }
};

static JitAllocator globalJitAllocator;

void* allocateExecutableMemory(size_t size) {
    return globalJitAllocator.allocateExecutable(size);
}

void deallocateExecutableMemory(void* ptr, size_t size) {
    globalJitAllocator.deallocateExecutable(ptr, size);
}

} // namespace ARM64Recompiler
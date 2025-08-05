// ARM64 Optimized Memory Pool Header
// High-performance memory management for PCSX2 recompiler

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace ARM64Recompiler {

// Memory statistics structure
struct MemoryStats {
    size_t microProgramMemory;
    size_t blockCacheMemory;
    size_t instructionMemory;
    size_t largeBufferMemory;
    size_t totalMemory;
};

// Optimized allocation functions for different component types
void* allocateMicroProgram();
void releaseMicroProgram(void* ptr);

void* allocateCodeBlock();
void releaseCodeBlock(void* ptr);

void* allocateInstruction();
void releaseInstruction(void* ptr);

void* allocateLargeBuffer();
void releaseLargeBuffer(void* ptr);

// ARM64 NEON-optimized memory operations
void fastMemoryClearing(void* ptr, size_t size);

// Memory usage tracking
MemoryStats getMemoryStats();

// RAII memory management classes
template<typename T>
class ManagedMicroProgram {
private:
    T* ptr;
    
public:
    ManagedMicroProgram() : ptr(static_cast<T*>(allocateMicroProgram())) {}
    ~ManagedMicroProgram() { if (ptr) releaseMicroProgram(ptr); }
    
    ManagedMicroProgram(const ManagedMicroProgram&) = delete;
    ManagedMicroProgram& operator=(const ManagedMicroProgram&) = delete;
    
    ManagedMicroProgram(ManagedMicroProgram&& other) noexcept : ptr(other.ptr) {
        other.ptr = nullptr;
    }
    
    ManagedMicroProgram& operator=(ManagedMicroProgram&& other) noexcept {
        if (this != &other) {
            if (ptr) releaseMicroProgram(ptr);
            ptr = other.ptr;
            other.ptr = nullptr;
        }
        return *this;
    }
    
    T* get() const { return ptr; }
    T* operator->() const { return ptr; }
    T& operator*() const { return *ptr; }
    explicit operator bool() const { return ptr != nullptr; }
};

template<typename T>
class ManagedCodeBlock {
private:
    T* ptr;
    
public:
    ManagedCodeBlock() : ptr(static_cast<T*>(allocateCodeBlock())) {}
    ~ManagedCodeBlock() { if (ptr) releaseCodeBlock(ptr); }
    
    ManagedCodeBlock(const ManagedCodeBlock&) = delete;
    ManagedCodeBlock& operator=(const ManagedCodeBlock&) = delete;
    
    ManagedCodeBlock(ManagedCodeBlock&& other) noexcept : ptr(other.ptr) {
        other.ptr = nullptr;
    }
    
    ManagedCodeBlock& operator=(ManagedCodeBlock&& other) noexcept {
        if (this != &other) {
            if (ptr) releaseCodeBlock(ptr);
            ptr = other.ptr;
            other.ptr = nullptr;
        }
        return *this;
    }
    
    T* get() const { return ptr; }
    T* operator->() const { return ptr; }
    T& operator*() const { return *ptr; }
    explicit operator bool() const { return ptr != nullptr; }
};

} // namespace ARM64Recompiler
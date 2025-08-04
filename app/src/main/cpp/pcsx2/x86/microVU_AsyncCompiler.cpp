// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "microVU_AsyncCompiler.h"
#include "microVU.h"
#include "Common.h"
#include "MTVU.h"
#include "VMManager.h"
#include "PerformanceMetrics.h"

#include <chrono>
#include <algorithm>

// Forward declaration from microVU.cpp
extern microProgram* mVUcreateProg(microVU& mVU, int startPC);

// Global async compiler instance
microVU_AsyncCompiler g_microVU_AsyncCompiler;

// Static configuration - DISABLED due to MTVU thread safety issues
std::atomic<bool> microVU_AsyncCompiler::s_asyncEnabled{false};

//------------------------------------------------------------------
// ARM64 Thread Affinity Optimization
//------------------------------------------------------------------

void microVU_AsyncCompiler::SetWorkerThreadAffinity(int threadIndex)
{
#ifdef __linux__
    // ARM64: Try to bind worker threads to performance cores
    // Most ARM64 devices have performance cores on higher CPU indices
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    
    // Get number of CPU cores
    int numCpus = std::thread::hardware_concurrency();
    if (numCpus <= 4)
    {
        // Small core count: use any available core except core 0 (main thread)
        int targetCore = (threadIndex + 1) % numCpus;
        CPU_SET(targetCore, &cpuset);
    }
    else
    {
        // Large core count: prefer performance cores (typically higher indices)
        // Leave cores 0-1 for main thread and MTVU, use cores 2+ for async compilation
        int targetCore = std::min(2 + threadIndex, numCpus - 1);
        CPU_SET(targetCore, &cpuset);
    }
    
    //pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    sched_setaffinity(pthread_self(), sizeof(cpu_set_t), &cpuset);
    
    // Set thread name for debugging
    char threadName[16];
    snprintf(threadName, sizeof(threadName), "mVU_Async_%d", threadIndex);
    pthread_setname_np(pthread_self(), threadName);
#endif
}

//------------------------------------------------------------------
// Constructor and Initialization
//------------------------------------------------------------------

microVU_AsyncCompiler::microVU_AsyncCompiler()
{
    // Constructor intentionally minimal - call Initialize() explicitly
}

microVU_AsyncCompiler::~microVU_AsyncCompiler()
{
    Shutdown();
}

void microVU_AsyncCompiler::Initialize()
{
    if (!s_asyncEnabled.load())
        return;
        
    Console.WriteLn("microVU AsyncCompiler: Initializing ARM64 asynchronous compilation system");
    
    // Reset state
    m_shutdown.store(false);
    m_paused.store(false);
    m_totalCompiled.store(0);
    m_cacheHits.store(0);
    m_totalCompileTime.store(0);
    m_prewarmActive.store(false);
    
    // Clear any existing queue
    ClearQueue();
    
    // Determine optimal worker thread count based on hardware
    int numWorkers = std::min(MAX_WORKER_THREADS, (int)std::thread::hardware_concurrency() - 2);
    numWorkers = std::max(1, numWorkers); // At least 1 worker
    
    Console.WriteLn("microVU AsyncCompiler: Starting %d worker threads", numWorkers);
    
    // Start worker threads
    m_workerThreads.reserve(numWorkers);
    for (int i = 0; i < numWorkers; ++i)
    {
        m_workerThreads.emplace_back([this, i]() { WorkerThreadMain(i); });
    }
    
    Console.WriteLn("microVU AsyncCompiler: Initialization complete");
}

void microVU_AsyncCompiler::Shutdown()
{
    if (m_workerThreads.empty())
        return;
        
    Console.WriteLn("microVU AsyncCompiler: Shutting down async compilation system");
    
    // Signal shutdown
    m_shutdown.store(true);
    m_queueCondition.notify_all();
    
    // Wait for all worker threads to finish
    for (auto& thread : m_workerThreads)
    {
        if (thread.joinable())
            thread.join();
    }
    m_workerThreads.clear();
    
    // Clear queue
    ClearQueue();
    
    Console.WriteLn("microVU AsyncCompiler: Shutdown complete. Compiled %d programs total", 
                   m_totalCompiled.load());
}

//------------------------------------------------------------------
// Worker Thread Implementation
//------------------------------------------------------------------

void microVU_AsyncCompiler::WorkerThreadMain(int threadIndex)
{
    SetWorkerThreadAffinity(threadIndex);
    
    Console.WriteLn("microVU AsyncCompiler: Worker thread %d started", threadIndex);
    m_activeWorkers.fetch_add(1);
    
    while (!m_shutdown.load())
    {
        AsyncCompileRequest request(nullptr, 0, nullptr);
        bool hasRequest = false;
        
        // Wait for work or shutdown signal
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCondition.wait(lock, [this] { 
                return m_shutdown.load() || (!m_compileQueue.empty() && !m_paused.load()); 
            });
            
            if (m_shutdown.load())
                break;
                
            if (!m_compileQueue.empty() && !m_paused.load())
            {
                request = m_compileQueue.top();
                m_compileQueue.pop();
                hasRequest = true;
            }
        }
        
        if (hasRequest)
        {
            // Check if compilation is still needed
            if (ShouldSkipCompilation(request))
            {
                m_cacheHits.fetch_add(1);
                continue;
            }
            
            // Perform compilation
            auto startTime = std::chrono::high_resolution_clock::now();
            
            microProgram* program = CompileProgramAsync(*request.mVU, request.startPC, request.progList);
            
            auto endTime = std::chrono::high_resolution_clock::now();
            auto compileTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            
            if (program)
            {
                UpdatePerformanceMetrics(compileTime.count(), false);
                
                // During pre-warm phase, add to tracking set
                if (request.isPrewarm && m_prewarmActive.load())
                {
                    std::lock_guard<std::mutex> prewarmLock(m_prewarmMutex);
                    m_prewarmPCs.insert(request.startPC);
                }
            }
        }
    }
    
    m_activeWorkers.fetch_sub(1);
    Console.WriteLn("microVU AsyncCompiler: Worker thread %d finished", threadIndex);
}

//------------------------------------------------------------------
// Compilation Functions
//------------------------------------------------------------------

microProgram* microVU_AsyncCompiler::CompileProgramAsync(microVU& mVU, int startPC, microProgramList* progList)
{
    // This function runs in worker thread - must be thread-safe
    
    // Check if program was already compiled by another thread
    if (progList && !progList->empty())
    {
        for (auto& prog : *progList)
        {
            if ((int)prog->startPC == startPC)
            {
                return prog; // Already compiled
            }
        }
    }
    
    // Perform actual compilation using existing microVU infrastructure
    // Note: This must be adapted to work safely from worker threads
    
//    try
//    {
        // Critical section: program creation must be synchronized
        static std::mutex s_compileMutex;
        std::lock_guard<std::mutex> lock(s_compileMutex);
        
        // Double-check pattern: verify program doesn't exist after acquiring lock
        if (progList && !progList->empty())
        {
            for (auto& prog : *progList)
            {
                if ((int)prog->startPC == startPC)
                {
                    return prog; // Another thread compiled it
                }
            }
        }
        
        // Create new program using existing mVUcreateProg function
        // This is now safe because we're under synchronization lock
        microProgram* program = mVUcreateProg(mVU, startPC);
        
        if (program && progList)
        {
            progList->push_front(program);
        }
        
        m_totalCompiled.fetch_add(1);
        return program;
//    }
//    catch (const std::exception& e)
//    {
//        Console.Error("microVU AsyncCompiler: Compilation failed for PC=0x%04x: %s", startPC, e.what());
//        return nullptr;
//    }
}

bool microVU_AsyncCompiler::ShouldSkipCompilation(const AsyncCompileRequest& request)
{
    // Check if request is too old (timeout)
    u64 currentTime = GetCurrentTimeMs();
    const u64 TIMEOUT_MS = 5000; // 5 second timeout for async requests
    
    if (!request.isPrewarm && (currentTime - request.requestTime) > TIMEOUT_MS)
    {
        return true; // Skip old requests
    }
    
    // Check if program already exists
    if (request.progList && !request.progList->empty())
    {
        for (const auto& prog : *request.progList)
        {
            if ((int)prog->startPC == request.startPC)
            {
                return true; // Already compiled
            }
        }
    }
    
    return false;
}

//------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------

void microVU_AsyncCompiler::QueueCompilation(microVU& mVU, int startPC, microProgramList* progList, u32 priority)
{
    if (!s_asyncEnabled.load() || !THREAD_VU1)
        return;
        
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        AsyncCompileRequest request(&mVU, startPC, progList, priority, false);
        request.requestTime = GetCurrentTimeMs();
        m_compileQueue.push(request);
    }
    m_queueCondition.notify_one();
}

microProgram* microVU_AsyncCompiler::GetOrQueueCompilation(microVU& mVU, int startPC, microProgramList* progList, bool waitIfNeeded)
{
    // First check if program already exists (fast path)
    if (progList && !progList->empty())
    {
        for (auto& prog : *progList)
        {
            if ((int)prog->startPC == startPC)
            {
                return prog;
            }
        }
    }
    
    if (!s_asyncEnabled.load() || !THREAD_VU1)
        return nullptr;
    
    // Queue compilation with high priority if waiting
    u32 priority = waitIfNeeded ? 90 : 70;
    QueueCompilation(mVU, startPC, progList, priority);
    
    if (waitIfNeeded)
    {
        // For urgent requests, wait a short time for compilation
        const int MAX_WAIT_MS = 50; // 50ms max wait to avoid blocking too long
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(MAX_WAIT_MS);
        
        while (std::chrono::steady_clock::now() < deadline)
        {
            // Check if program was compiled
            if (progList && !progList->empty())
            {
                for (auto& prog : *progList)
                {
                    if ((int)prog->startPC == startPC)
                    {
                        return prog;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    return nullptr; // Will fallback to synchronous compilation
}

//------------------------------------------------------------------
// Cache Pre-warming
//------------------------------------------------------------------

void microVU_AsyncCompiler::StartPrewarmPhase()
{
    if (!s_asyncEnabled.load())
        return;
        
    Console.WriteLn("microVU AsyncCompiler: Starting cache pre-warm phase");
    
    m_prewarmActive.store(true);
    {
        std::lock_guard<std::mutex> lock(m_prewarmMutex);
        m_prewarmPCs.clear();
    }
    
    // Resume compilation if paused
    ResumeCompilation();
}

void microVU_AsyncCompiler::StopPrewarmPhase()
{
    if (!m_prewarmActive.load())
        return;
        
    Console.WriteLn("microVU AsyncCompiler: Stopping cache pre-warm phase. Pre-warmed %zu programs", 
                   m_prewarmPCs.size());
    
    m_prewarmActive.store(false);
    
    // Clear pre-warm tracking
    {
        std::lock_guard<std::mutex> lock(m_prewarmMutex);
        m_prewarmPCs.clear();
    }
}

void microVU_AsyncCompiler::QueuePrewarmCompilation(microVU& mVU, int startPC, microProgramList* progList)
{
    if (!s_asyncEnabled.load() || !m_prewarmActive.load())
        return;
    
    // Check if already pre-warmed
    {
        std::lock_guard<std::mutex> lock(m_prewarmMutex);
        if (m_prewarmPCs.count(startPC) > 0)
            return; // Already queued for pre-warming
    }
    
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        AsyncCompileRequest request(&mVU, startPC, progList, 30, true);
        request.requestTime = GetCurrentTimeMs();
        m_compileQueue.push(request);
    }
    m_queueCondition.notify_one();
}

//------------------------------------------------------------------
// Utility Functions
//------------------------------------------------------------------

u64 microVU_AsyncCompiler::GetCurrentTimeMs()
{
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count();
}

void microVU_AsyncCompiler::UpdatePerformanceMetrics(u64 compileTimeMs, bool cacheHit)
{
    if (cacheHit)
    {
        m_cacheHits.fetch_add(1);
    }
    else
    {
        m_totalCompiled.fetch_add(1);
        m_totalCompileTime.fetch_add(compileTimeMs);
    }
}

void microVU_AsyncCompiler::ClearQueue()
{
    std::lock_guard<std::mutex> lock(m_queueMutex);
    // Clear the priority queue by creating a new empty one
    std::priority_queue<AsyncCompileRequest> empty;
    m_compileQueue.swap(empty);
}

bool microVU_AsyncCompiler::IsQueueEmpty() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_queueMutex));
    return m_compileQueue.empty();
}

size_t microVU_AsyncCompiler::GetQueueSize() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_queueMutex));
    return m_compileQueue.size();
}

microVU_AsyncCompiler::CompilerStats microVU_AsyncCompiler::GetStats() const
{
    CompilerStats stats;
    stats.totalCompiled = m_totalCompiled.load();
    stats.cacheHits = m_cacheHits.load();
    stats.pendingRequests = GetQueueSize();
    
    u64 totalTime = m_totalCompileTime.load();
    u32 totalCompiled = m_totalCompiled.load();
    stats.averageCompileTimeMs = totalCompiled > 0 ? (float)totalTime / totalCompiled : 0.0f;
    stats.isPrewarmActive = m_prewarmActive.load();
    
    return stats;
}

//------------------------------------------------------------------
// Static Configuration
//------------------------------------------------------------------

bool microVU_AsyncCompiler::IsAsyncCompilationEnabled()
{
    return s_asyncEnabled.load();
}

void microVU_AsyncCompiler::SetAsyncCompilationEnabled(bool enabled)
{
    s_asyncEnabled.store(enabled);
    
    if (!enabled)
    {
        g_microVU_AsyncCompiler.Shutdown();
    }
    else if (VMManager::GetState() != VMState::Shutdown)
    {
        g_microVU_AsyncCompiler.Initialize();
    }
}
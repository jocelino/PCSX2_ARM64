// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "microVU.h"
#include "common/Threading.h"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <unordered_set>

//------------------------------------------------------------------
// ARM64 Asynchronous microVU1 Compilation System
//------------------------------------------------------------------
// 
// This system addresses performance stutters during cutscenes caused by
// synchronous microVU1 program compilation. It implements:
//
// 1. Background compilation queue to prevent main thread blocking
// 2. Priority-based compilation (frequently used programs first)
// 3. Cache pre-warming during loading screens
// 4. ARM64-optimized thread affinity for performance cores
// 5. Integration with existing MTVU system
//
//------------------------------------------------------------------

struct AsyncCompileRequest
{
    microVU* mVU;
    int startPC;
    microProgramList* progList;
    u32 priority;           // Higher = more urgent (0-100)
    u64 requestTime;        // For timeout handling
    bool isPrewarm;         // Pre-warming vs on-demand
    
    AsyncCompileRequest(microVU* vu, int pc, microProgramList* list, u32 prio = 50, bool prewarm = false)
        : mVU(vu), startPC(pc), progList(list), priority(prio), isPrewarm(prewarm), requestTime(0)
    {
        // requestTime will be set in actual implementation
    }
    
    // Priority queue ordering (higher priority first)
    bool operator<(const AsyncCompileRequest& other) const
    {
        if (priority != other.priority)
            return priority < other.priority;
        return requestTime > other.requestTime; // Older requests first for same priority
    }
};

class microVU_AsyncCompiler
{
private:
    // Compilation queue and thread management
    std::priority_queue<AsyncCompileRequest> m_compileQueue;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCondition;
    std::atomic<bool> m_shutdown{false};
    std::atomic<bool> m_paused{false};
    
    // Worker threads (ARM64: use multiple cores during loading)
    std::vector<std::thread> m_workerThreads;
    std::atomic<int> m_activeWorkers{0};
    static constexpr int MAX_WORKER_THREADS = 3; // Leave cores for main thread + MTVU
    
    // Performance monitoring
    std::atomic<u32> m_totalCompiled{0};
    std::atomic<u32> m_cacheHits{0};
    std::atomic<u64> m_totalCompileTime{0};
    
    // Cache pre-warming state
    std::atomic<bool> m_prewarmActive{false};
    std::unordered_set<u32> m_prewarmPCs; // Track prewarmed PCs to avoid duplicates
    std::mutex m_prewarmMutex;
    
    // ARM64 thread affinity optimization
    void SetWorkerThreadAffinity(int threadIndex);
    
    // Worker thread main loop
    void WorkerThreadMain(int threadIndex);
    
    // Compilation functions
    microProgram* CompileProgramAsync(microVU& mVU, int startPC, microProgramList* progList);
    bool ShouldSkipCompilation(const AsyncCompileRequest& request);
    
    // Utility functions
    static u64 GetCurrentTimeMs();
    void UpdatePerformanceMetrics(u64 compileTimeMs, bool cacheHit);
    
public:
    microVU_AsyncCompiler();
    ~microVU_AsyncCompiler();
    
    // Initialization and shutdown
    void Initialize();
    void Shutdown();
    
    // Main compilation interface
    void QueueCompilation(microVU& mVU, int startPC, microProgramList* progList, u32 priority = 50);
    microProgram* GetOrQueueCompilation(microVU& mVU, int startPC, microProgramList* progList, bool waitIfNeeded = false);
    
    // Cache pre-warming for loading screens
    void StartPrewarmPhase();
    void StopPrewarmPhase();
    void QueuePrewarmCompilation(microVU& mVU, int startPC, microProgramList* progList);
    
    // Performance and monitoring
    void PauseCompilation() { m_paused.store(true); }
    void ResumeCompilation() { m_paused.store(false); m_queueCondition.notify_all(); }
    
    // Statistics
    struct CompilerStats
    {
        u32 totalCompiled;
        u32 cacheHits;
        u32 pendingRequests;
        float averageCompileTimeMs;
        bool isPrewarmActive;
    };
    CompilerStats GetStats() const;
    
    // Queue management
    void ClearQueue();
    bool IsQueueEmpty() const;
    size_t GetQueueSize() const;
    
    // Integration with existing systems
    static bool IsAsyncCompilationEnabled();
    static void SetAsyncCompilationEnabled(bool enabled);
    
private:
    static std::atomic<bool> s_asyncEnabled;
};

// Global async compiler instance
extern microVU_AsyncCompiler g_microVU_AsyncCompiler;

//------------------------------------------------------------------
// Integration macros for existing microVU code
//------------------------------------------------------------------

#define mVU_ASYNC_COMPILE_ENABLED() (microVU_AsyncCompiler::IsAsyncCompilationEnabled() && THREAD_VU1)

// Replace synchronous mVUcreateProg calls during cache misses
#define mVU_GET_OR_COMPILE_ASYNC(mVU, startPC, progList) \
    (mVU_ASYNC_COMPILE_ENABLED() ? \
     g_microVU_AsyncCompiler.GetOrQueueCompilation(mVU, startPC, progList, false) : \
     nullptr)

// High priority compilation for immediate execution
#define mVU_COMPILE_URGENT(mVU, startPC, progList) \
    (mVU_ASYNC_COMPILE_ENABLED() ? \
     g_microVU_AsyncCompiler.GetOrQueueCompilation(mVU, startPC, progList, true) : \
     nullptr)

// Pre-warming during loading screens
#define mVU_PREWARM_PROGRAM(mVU, startPC, progList) \
    do { \
        if (mVU_ASYNC_COMPILE_ENABLED()) \
            g_microVU_AsyncCompiler.QueuePrewarmCompilation(mVU, startPC, progList); \
    } while(0)
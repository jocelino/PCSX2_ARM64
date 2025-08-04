// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "microVU_PrewarmSystem.h"
#include "microVU_AsyncCompiler.h"
#include "Common.h"
#include "MTVU.h"

#include <algorithm>
#include <chrono>

// Global pre-warm system instance
microVU_PrewarmSystem g_microVU_PrewarmSystem;

//------------------------------------------------------------------
// Constructor and Utility Functions
//------------------------------------------------------------------

microVU_PrewarmSystem::microVU_PrewarmSystem()
{
    m_lastActivity = GetCurrentTimeMs();
}

u64 microVU_PrewarmSystem::GetCurrentTimeMs() const
{
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count();
}

//------------------------------------------------------------------
// Loading Screen Detection
//------------------------------------------------------------------

void microVU_PrewarmSystem::UpdateLoadingDetection()
{
    bool wasLoading = m_loadingDetected.load();
    bool isLoadingNow = (m_framesSinceActivity >= LOADING_DETECTION_FRAMES);
    
    if (isLoadingNow != wasLoading)
    {
        m_loadingDetected.store(isLoadingNow);
        
        if (isLoadingNow)
        {
            Console.WriteLn("microVU PrewarmSystem: Loading screen detected - starting pre-warm phase");
            g_microVU_AsyncCompiler.StartPrewarmPhase();
        }
        else
        {
            Console.WriteLn("microVU PrewarmSystem: Loading screen ended - stopping pre-warm phase");
            g_microVU_AsyncCompiler.StopPrewarmPhase();
        }
    }
}

void microVU_PrewarmSystem::OnSignificantActivity()
{
    if (!m_enabled.load())
        return;
        
    m_lastActivity.store(GetCurrentTimeMs());
    m_framesSinceActivity.store(0);
    
    // Update loading detection immediately
    UpdateLoadingDetection();
}

void microVU_PrewarmSystem::OnFrameComplete()
{
    if (!m_enabled.load())
        return;
        
    // Increment frames since last significant activity
    u32 framesSince = m_framesSinceActivity.fetch_add(1) + 1;
    
    // Check for loading screen state change every few frames to avoid overhead
    if (framesSince % 10 == 0)
    {
        UpdateLoadingDetection();
    }
}

//------------------------------------------------------------------
// Program Usage Tracking
//------------------------------------------------------------------

void microVU_PrewarmSystem::RecordProgramUsage(u32 startPC, u32 executionCycles)
{
    std::lock_guard<std::mutex> lock(m_statsMutex);
    
    ProgramStats& stats = m_programStats[startPC];
    stats.usageCount++;
    stats.lastUsed = GetCurrentTimeMs();
    
    // Update average execution time (simple moving average)
    if (stats.executionTime == 0)
        stats.executionTime = executionCycles;
    else
        stats.executionTime = (stats.executionTime * 3 + executionCycles) / 4;
}

void microVU_PrewarmSystem::OnProgramExecuted(u32 startPC, u32 executionCycles)
{
    if (!m_enabled.load())
        return;
        
    RecordProgramUsage(startPC, executionCycles);
    
    // Any program execution counts as activity
    OnSignificantActivity();
}

std::vector<u32> microVU_PrewarmSystem::GetCandidatesForPrewarming() const
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_statsMutex));
    
    std::vector<std::pair<u32, u32>> candidates; // PC, priority score
    
    for (const auto& pair : m_programStats)
    {
        const u32 pc = pair.first;
        const ProgramStats& stats = pair.second;
        
        // Only consider programs used frequently enough
        if (stats.usageCount < MIN_USAGE_FOR_PREWARM)
            continue;
            
        // Calculate priority score based on usage frequency and execution time
        // Higher usage count and longer execution time = higher priority
        u32 priorityScore = stats.usageCount * 10;
        if (stats.executionTime > 1000) // Long-running programs get bonus
            priorityScore += 50;
            
        candidates.emplace_back(pc, priorityScore);
    }
    
    // Sort by priority score (descending)
    std::sort(candidates.begin(), candidates.end(), 
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    // Extract PCs, limited to max count
    std::vector<u32> result;
    result.reserve(std::min((size_t)MAX_PREWARM_PROGRAMS, candidates.size()));
    
    for (size_t i = 0; i < std::min((size_t)MAX_PREWARM_PROGRAMS, candidates.size()); ++i)
    {
        result.push_back(candidates[i].first);
    }
    
    return result;
}

//------------------------------------------------------------------
// Pre-warming Implementation
//------------------------------------------------------------------

void microVU_PrewarmSystem::TriggerPrewarmIfNeeded(microVU& mVU)
{
    if (!m_enabled.load() || mVU.index != 1 || !THREAD_VU1)
        return;
        
    // Only prewarm during loading screens
    if (!m_loadingDetected.load())
        return;
        
    // Get programs that should be pre-warmed
    auto candidates = GetCandidatesForPrewarming();
    
    if (candidates.empty())
        return;
        
    Console.WriteLn("microVU PrewarmSystem: Pre-warming %zu programs during loading screen", 
                   candidates.size());
    
    // Queue pre-warm compilation for each candidate
    for (u32 startPC : candidates)
    {
        u32 pc_index = startPC >> 3; // Convert to index
        if (pc_index < mVU.progSize)
        {
            microProgramList* progList = mVU.prog.prog[pc_index];
            if (progList)
            {
                g_microVU_AsyncCompiler.QueuePrewarmCompilation(mVU, startPC, progList);
                m_prewarmedThisSession.fetch_add(1);
            }
        }
    }
}

void microVU_PrewarmSystem::ForcePrewarmSession(microVU& mVU)
{
    if (!m_enabled.load() || mVU.index != 1)
        return;
        
    Console.WriteLn("microVU PrewarmSystem: Forcing pre-warm session");
    
    // Temporarily enable pre-warming
    bool wasLoading = m_loadingDetected.load();
    m_loadingDetected.store(true);
    
    g_microVU_AsyncCompiler.StartPrewarmPhase();
    TriggerPrewarmIfNeeded(mVU);
    
    // Schedule stop after a short delay (will be handled by async system)
    std::thread([this, wasLoading]() {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        g_microVU_AsyncCompiler.StopPrewarmPhase();
        m_loadingDetected.store(wasLoading);
    }).detach();
}

//------------------------------------------------------------------
// Statistics and Monitoring
//------------------------------------------------------------------

microVU_PrewarmSystem::PrewarmStats microVU_PrewarmSystem::GetStats() const
{
    PrewarmStats stats;
    
    {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_statsMutex));
        stats.totalTrackedPrograms = m_programStats.size();
    }
    
    stats.prewarmedThisSession = m_prewarmedThisSession.load();
    stats.isLoadingDetected = m_loadingDetected.load();
    stats.framesSinceActivity = m_framesSinceActivity.load();
    
    return stats;
}
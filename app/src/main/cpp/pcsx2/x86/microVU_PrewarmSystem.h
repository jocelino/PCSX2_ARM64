// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "microVU.h"
#include <unordered_set>
#include <unordered_map>
#include <mutex>
#include <atomic>

//------------------------------------------------------------------
// ARM64 Cache Pre-warming System for Loading Screens
//------------------------------------------------------------------
//
// This system detects loading screen patterns and proactively compiles
// frequently used microVU1 programs to reduce stutters during gameplay
//
//------------------------------------------------------------------

class microVU_PrewarmSystem
{
private:
    // Loading detection state
    std::atomic<bool> m_loadingDetected{false};
    std::atomic<u64> m_lastActivity{0};
    std::atomic<u32> m_framesSinceActivity{0};
    
    // Program usage tracking
    struct ProgramStats
    {
        u32 usageCount{0};
        u64 lastUsed{0};
        u32 executionTime{0}; // Average execution time in cycles
    };
    
    std::unordered_map<u32, ProgramStats> m_programStats; // PC -> stats
    std::mutex m_statsMutex;
    
    // Pre-warming configuration
    static constexpr u32 MIN_USAGE_FOR_PREWARM = 3;  // Must be used at least 3 times
    static constexpr u32 LOADING_DETECTION_FRAMES = 60; // 60 frames of low activity = loading
    static constexpr u32 MAX_PREWARM_PROGRAMS = 50;  // Limit to prevent excessive compilation
    
    // Utility functions
    u64 GetCurrentTimeMs() const;
    bool IsLoadingScreenActive() const;
    void UpdateLoadingDetection();
    
    // Program analysis
    void RecordProgramUsage(u32 startPC, u32 executionCycles);
    std::vector<u32> GetCandidatesForPrewarming() const;
    
public:
    microVU_PrewarmSystem();
    ~microVU_PrewarmSystem() = default;
    
    // Main interface called from microVU execution
    void OnProgramExecuted(u32 startPC, u32 executionCycles);
    void OnFrameComplete();
    
    // Loading screen detection
    void OnSignificantActivity(); // Called when significant game activity occurs
    bool IsInLoadingScreen() const { return m_loadingDetected.load(); }
    
    // Pre-warming control
    void TriggerPrewarmIfNeeded(microVU& mVU);
    void ForcePrewarmSession(microVU& mVU); // Manual trigger
    
    // Statistics and monitoring
    struct PrewarmStats
    {
        u32 totalTrackedPrograms;
        u32 prewarmedThisSession;
        bool isLoadingDetected;
        u32 framesSinceActivity;
    };
    PrewarmStats GetStats() const;
    
    // Configuration
    void SetEnabled(bool enabled) { m_enabled = enabled; }
    bool IsEnabled() const { return m_enabled; }
    
private:
    std::atomic<bool> m_enabled{true};
    std::atomic<u32> m_prewarmedThisSession{0};
};

// Global pre-warm system instance
extern microVU_PrewarmSystem g_microVU_PrewarmSystem;

//------------------------------------------------------------------
// Integration macros for automatic loading detection
//------------------------------------------------------------------

// Call this when a microVU1 program executes
#define mVU_RECORD_PROGRAM_USAGE(startPC, cycles) \
    do { \
        if (g_microVU_PrewarmSystem.IsEnabled()) \
            g_microVU_PrewarmSystem.OnProgramExecuted(startPC, cycles); \
    } while(0)

// Call this at the end of each frame
#define mVU_FRAME_COMPLETE() \
    do { \
        if (g_microVU_PrewarmSystem.IsEnabled()) { \
            g_microVU_PrewarmSystem.OnFrameComplete(); \
            g_microVU_PrewarmSystem.TriggerPrewarmIfNeeded(microVU1); \
        } \
    } while(0)

// Call this when significant game events occur (input, audio changes, etc.)
#define mVU_SIGNIFICANT_ACTIVITY() \
    do { \
        if (g_microVU_PrewarmSystem.IsEnabled()) \
            g_microVU_PrewarmSystem.OnSignificantActivity(); \
    } while(0)
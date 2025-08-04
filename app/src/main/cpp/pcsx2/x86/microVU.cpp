// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "microVU.h"
#include "microVU_AsyncCompiler.h"

#include "common/AlignedMalloc.h"
#include "common/Perf.h"
#include "common/StringUtil.h"
#include <shared_mutex>
#include <atomic>

alignas(16) vuRegistersPack g_vuRegistersPack;

//------------------------------------------------------------------
// ARM64 Performance Optimization - Object Pools
//------------------------------------------------------------------

// Thread-safe memory pool for microProgram structures (64-byte aligned)
template<typename T, size_t Alignment>
class alignedPool
{
private:
    std::vector<T*> freeList;
    std::vector<void*> allocatedChunks;
    size_t chunkSize;
    size_t itemsPerChunk;
    mutable std::mutex poolMutex;  // Thread safety for pool operations
    
public:
    alignedPool(size_t initialChunkSize = 64) : chunkSize(initialChunkSize)
    {
        itemsPerChunk = chunkSize;
        reserve();
    }
    
    ~alignedPool()
    {
        std::lock_guard<std::mutex> lock(poolMutex);
        for (void* chunk : allocatedChunks)
            _aligned_free(chunk);
    }
    
    T* acquire()
    {
        std::lock_guard<std::mutex> lock(poolMutex);
        if (freeList.empty())
            reserve();
        
        T* item = freeList.back();
        freeList.pop_back();
        return item;
    }
    
    void release(T* item)
    {
        if (item)
        {
            std::lock_guard<std::mutex> lock(poolMutex);
            freeList.push_back(item);
        }
    }
    
private:
    void reserve()
    {
        // Allocate chunk with ARM64-optimized alignment
        void* chunk = _aligned_malloc(sizeof(T) * itemsPerChunk, Alignment);
        if (!chunk)
            pxFailRel("Failed to allocate memory pool chunk");
        
        allocatedChunks.push_back(chunk);
        
        // Add all items in chunk to free list
        T* items = static_cast<T*>(chunk);
        for (size_t i = 0; i < itemsPerChunk; ++i)
        {
            freeList.push_back(&items[i]);
        }
        
        // Double chunk size for next allocation (exponential growth)
        if (chunkSize < 512)  // Cap at reasonable size
            chunkSize *= 2;
    }
};

// Thread-safe object pools for frequently allocated structures
static alignedPool<microProgram, 64> g_microProgramPool(32);      // 64-byte aligned, start with 32 items
static alignedPool<microBlockLink, 32> g_microBlockLinkPool(64);  // 32-byte aligned, start with 64 items

static std::shared_mutex g_programCacheMutex;

// ARM64 Performance Optimization - Object Pool Access Functions
microBlockLink* mVUacquireBlockLink()
{
	microBlockLink* link = g_microBlockLinkPool.acquire();
	// ARM64 optimization: Clear only essential fields, avoid full struct zeroing
	// microBlock.jumpCache is set to nullptr in caller, microBlock fields are overwritten
	link->next = nullptr;
	// Note: microBlock contents will be copied from caller, so no need to clear
	return link;
}

void mVUreleaseBlockLink(microBlockLink* link)
{
	if (link)
	{
		// Clean up any allocated resources
		safe_delete_array(link->block.jumpCache);
		g_microBlockLinkPool.release(link);
	}
}

//------------------------------------------------------------------
// Micro VU - Main Functions
//------------------------------------------------------------------

// Only run this once per VU! ;)
void mVUinit(microVU& mVU, uint vuIndex)
{
	std::memset(&mVU.prog, 0, sizeof(mVU.prog));

	mVU.index        =  vuIndex;
	mVU.cop2         =  0;
	mVU.vuMemSize    = (mVU.index ? 0x4000 : 0x1000);
	mVU.microMemSize = (mVU.index ? 0x4000 : 0x1000);
	mVU.progSize     = (mVU.index ? 0x4000 : 0x1000) / 4;
	mVU.progMemMask  =  mVU.progSize-1;
	mVU.cache        = vuIndex ? SysMemory::GetVU1Rec() : SysMemory::GetVU0Rec();
	mVU.prog.x86end  = (vuIndex ? SysMemory::GetVU1RecEnd() : SysMemory::GetVU0RecEnd()) - (mVUcacheSafeZone * _1mb);

	mVU.regAlloc.reset(new microRegAlloc(mVU.index));
}

// Resets Rec Data
void mVUreset(microVU& mVU, bool resetReserve)
{
	if (THREAD_VU1)
	{
		DevCon.Warning("mVU Reset");
		// If MTVU is toggled on during gameplay we need to flush the running VU1 program, else it gets in a mess
		if (VU0.VI[REG_VPU_STAT].UL & 0x100)
		{
			CpuVU1->Execute(vu1RunCycles);
		}
		VU0.VI[REG_VPU_STAT].UL &= ~0x100;
	}

//	xSetPtr(mVU.cache);
    armSetAsmPtr(mVU.cache, mVU.index ? HostMemoryMap::mVU1recSize : HostMemoryMap::mVU0recSize, nullptr);

	mVUdispatcherAB(mVU);
	mVUdispatcherCD(mVU);
	mVUGenerateWaitMTVU(mVU);
	mVUGenerateCopyPipelineState(mVU);
	mVUGenerateCompareState(mVU);

	mVU.regs().nextBlockCycles = 0;
	memset(&mVU.prog.lpState, 0, sizeof(mVU.prog.lpState));
	mVU.profiler.Reset(mVU.index);

	// Program Variables
	mVU.prog.cleared  =  1;
	mVU.prog.isSame   = -1;
	mVU.prog.cur      = NULL;
	mVU.prog.total    =  0;
	mVU.prog.curFrame =  0;

	// Setup Dynarec Cache Limits for Each Program
//	mVU.prog.x86start = xGetAlignedCallTarget();
    armAlignAsmPtr();
	mVU.prog.x86ptr   = mVU.prog.x86start;

    u32 i, e = (mVU.progSize >> 1); // mVU.progSize / 2
	for ( i = 0; i < e; ++i)
	{
		if (!mVU.prog.prog[i])
		{
			mVU.prog.prog[i] = new std::deque<microProgram*>();
			continue;
		}
		auto it(mVU.prog.prog[i]->begin());
		for (; it != mVU.prog.prog[i]->end(); ++it)
		{
			mVUdeleteProg(mVU, it[0]);
		}
		mVU.prog.prog[i]->clear();
        mVU.prog.quick[i].block = NULL;
        mVU.prog.quick[i].prog = NULL;
	}
}

// Free Allocated Resources
void mVUclose(microVU& mVU)
{
	// Delete Programs and Block Managers
    u32 i, e = (mVU.progSize >> 1); // mVU.progSize / 2
	for (i = 0; i < e; ++i)
	{
		if (!mVU.prog.prog[i])
			continue;
		auto it(mVU.prog.prog[i]->begin());
		for (; it != mVU.prog.prog[i]->end(); ++it)
		{
			mVUdeleteProg(mVU, it[0]);
		}
		safe_delete(mVU.prog.prog[i]);
	}
}

// Clears Block Data in specified range
__fi void mVUclear(mV, u32 addr, u32 size)
{
    if (!mVU.prog.cleared)
    {
        mVU.prog.cleared = 1; // Next execution searches/creates a new microprogram
        // ARM64 optimization: Use more efficient clearing for frequently called function
        std::memset(&mVU.prog.lpState, 0, sizeof(mVU.prog.lpState)); // Clear pipeline state
        // Zero-initialize quick array with ARM64-optimized memset
        const size_t quickArraySize = (mVU.progSize >> 1) * sizeof(microProgramQuick);
        std::memset(mVU.prog.quick, 0, quickArraySize);
    }
}

//------------------------------------------------------------------
// Micro VU - Private Functions
//------------------------------------------------------------------

// Deletes a program
__ri void mVUdeleteProg(microVU& mVU, microProgram*& prog)
{
    u32 i, e = (mVU.progSize >> 1); // mVU.progSize / 2
	for (i = 0; i < e; ++i)
	{
		safe_delete(prog->block[i]);
	}
	safe_delete(prog->ranges);
	// ARM64 optimization: Return to object pool instead of aligned_free
	g_microProgramPool.release(prog);
	prog = nullptr;
}

// Creates a new Micro Program
__ri microProgram* mVUcreateProg(microVU& mVU, int startPC)
{
	// ARM64 optimization: Use object pool for microProgram allocation
	auto* prog = g_microProgramPool.acquire();
	// ARM64 optimization: Zero only the block array and data, avoid redundant clearing
	memset(prog->data, 0, sizeof(prog->data));
	memset(prog->block, 0, sizeof(prog->block));
	// Set fields directly without additional clearing
	prog->idx = mVU.prog.total++;
	prog->ranges = new std::deque<microRange>();
	prog->startPC = startPC;
	if(doWholeProgCompare)
		mVUcacheProg(mVU, *prog); // Cache Micro Program
	
	// ARM64 Performance: Reduce logging overhead during compilation bursts
	// Only log every 10th program during heavy compilation to reduce stutter
	static u32 logCounter = 0;
	if (mVU.index == 1 && (++logCounter % 10 == 0)) // VU1 only, every 10th
	{
		double cacheSize = (double)((uptr)mVU.prog.x86end - (uptr)mVU.prog.x86start);
		double cacheUsed = ((double)((uptr)mVU.prog.x86ptr - (uptr)mVU.prog.x86start)) / (double)_1mb;
		double cachePerc = ((double)((uptr)mVU.prog.x86ptr - (uptr)mVU.prog.x86start)) / cacheSize * 100;
		DevCon.WriteLn(Color_Orange, "microVU1: Compiled %d programs, Cache=%3.3f%% [%3.1fmb]",
			logCounter, cachePerc, cacheUsed);
	}
	return prog;
}

// Caches Micro Program
__ri void mVUcacheProg(microVU& mVU, microProgram& prog)
{
	if (!doWholeProgCompare)
	{
		auto cmpOffset = [&](void* x) { return (u8*)x + mVUrange.start; };
		memcpy(cmpOffset(prog.data), cmpOffset(mVU.regs().Micro), (mVUrange.end - mVUrange.start));
	}
	else
	{
		if (!mVU.index)
			memcpy(prog.data, mVU.regs().Micro, 0x1000);
		else
			memcpy(prog.data, mVU.regs().Micro, 0x4000);
	}
	mVUdumpProg(mVU, prog);
}

// Generate Hash for partial program based on compiled ranges...
u64 mVUrangesHash(microVU& mVU, microProgram& prog)
{
	union
	{
		u64 v64;
		u32 v32[2];
	} hash = {0};

	std::deque<microRange>::const_iterator it(prog.ranges->begin());
    int i, s, e;
	for (; it != prog.ranges->end(); ++it)
	{
		if ((it[0].start < 0) || (it[0].end < 0))
		{
			DevCon.Error("microVU%d: Negative Range![%d][%d]", mVU.index, it[0].start, it[0].end);
		}
        s = it[0].start >> 2; // it[0].start / 4
        e = it[0].end >> 2;   // it[0].end / 4
		for (i = s; i < e; ++i)
		{
			hash.v32[0] -= prog.data[i];
			hash.v32[1] ^= prog.data[i];
		}
	}
	return hash.v64;
}

// Prints the ratio of unique programs to total programs
void mVUprintUniqueRatio(microVU& mVU)
{
	std::vector<u64> v;
    u32 pc, e = mProgSize >> 1; // mProgSize / 2
	for (pc = 0; pc < e; ++pc)
	{
		microProgramList* list = mVU.prog.prog[pc];
		if (!list)
			continue;
		auto it(list->begin());
		for (; it != list->end(); ++it)
		{
			v.push_back(mVUrangesHash(mVU, *it[0]));
		}
	}
	u32 total = v.size();
	sortVector(v);
	makeUnique(v);
	if (!total)
		return;
	DevCon.WriteLn("%d / %d [%3.1f%%]", v.size(), total, 100. - (double)v.size() / (double)total * 100.);
}

// Compare Cached microProgram to mVU.regs().Micro
__fi bool mVUcmpProg(microVU& mVU, microProgram& prog)
{
	if (doWholeProgCompare)
	{
		if (memcmp((u8*)prog.data, mVU.regs().Micro, mVU.microMemSize))
			return false;
	}
	else
	{
		for (const auto& range : *prog.ranges)
		{
#if defined(PCSX2_DEVBUILD) || defined(_DEBUG)
			if ((range.start < 0) || (range.end < 0))
				DevCon.Error("microVU%d: Negative Range![%d][%d]", mVU.index, range.start, range.end);
#endif
			auto cmpOffset = [&](void* x) { return (u8*)x + range.start; };

			if (memcmp(cmpOffset(prog.data), cmpOffset(mVU.regs().Micro), (range.end - range.start)))
				return false;
		}
	}
	mVU.prog.cleared = 0;
	mVU.prog.cur = &prog;
	mVU.prog.isSame = doWholeProgCompare ? 1 : -1;
	return true;
}

// Thread-safe search for Cached Micro Program and sets prog.cur to it (returns entry-point to program)
_mVUt __fi void* mVUsearchProg(u32 startPC, uptr pState)
{
	microVU& mVU = mVUx;

    u32 start_pc_8 = startPC >> 3; // startPC / 8
    u32 regs_start_pc_8 = mVU.regs().start_pc >> 3; // mVU.regs().start_pc / 8

	std::shared_lock<std::shared_mutex> readLock(g_programCacheMutex);
	
	microProgramQuick& quick = mVU.prog.quick[regs_start_pc_8];
	microProgramList*  list  = mVU.prog.prog [regs_start_pc_8];

	if (!quick.prog) // If null, we need to search for new program
	{
		readLock.unlock();
		
		std::unique_lock<std::shared_mutex> writeLock(g_programCacheMutex);
		
		if (quick.prog)
		{
			mVU.prog.isSame = -1;
			mVU.prog.cur = quick.prog;
			quick.block = mVU.prog.cur->block[start_pc_8];
			
			if (quick.block == nullptr)
			{
				void* entryPoint = mVUblockFetch(mVU, startPC, pState);
				return entryPoint;
			}
			return mVUentryGet(mVU, quick.block, startPC, pState);
		}
		
		auto it(list->begin());
		for (; it != list->end(); ++it)
		{
			bool b = mVUcmpProg(mVU, *it[0]);

			if (b)
			{
				quick.block = it[0]->block[start_pc_8];
				quick.prog  = it[0];
				list->erase(it);
				list->push_front(quick.prog);

				// Sanity check, in case for some reason the program compilation aborted half way through (JALR for example)
				if (quick.block == nullptr)
				{
					void* entryPoint = mVUblockFetch(mVU, startPC, pState);
					return entryPoint;
				}
				return mVUentryGet(mVU, quick.block, startPC, pState);
			}
		}

		// If cleared and program not found, make a new program instance
		mVU.prog.cleared = 0;
		mVU.prog.isSame  = 1;
		
		mVU.prog.cur = mVUcreateProg(mVU, regs_start_pc_8);
		list->push_front(mVU.prog.cur);
		
		void* entryPoint = mVUblockFetch(mVU,  startPC, pState);
		quick.block      = mVU.prog.cur->block[start_pc_8];
		quick.prog       = mVU.prog.cur;
		//mVUprintUniqueRatio(mVU);
		return entryPoint;
	}

	// If list.quick, then we've already found and recompiled the program ;)
	mVU.prog.isSame = -1;
	mVU.prog.cur = quick.prog;
	// Because the VU's can now run in sections and not whole programs at once
	// we need to set the current block so it gets the right program back
	quick.block = mVU.prog.cur->block[start_pc_8];

	// Sanity check, in case for some reason the program compilation aborted half way through
	if (quick.block == nullptr)
	{
		// Release read lock before potential compilation
		readLock.unlock();
		void* entryPoint = mVUblockFetch(mVU, startPC, pState);
		return entryPoint;
	}
	return mVUentryGet(mVU, quick.block, startPC, pState);
}

//------------------------------------------------------------------
// recMicroVU0 / recMicroVU1
//------------------------------------------------------------------

recMicroVU0 CpuMicroVU0;
recMicroVU1 CpuMicroVU1;

recMicroVU0::recMicroVU0() { m_Idx = 0; IsInterpreter = false; }
recMicroVU1::recMicroVU1() { m_Idx = 1; IsInterpreter = false; }

void recMicroVU0::Reserve()
{
	mVUinit(microVU0, 0);
}
void recMicroVU1::Reserve()
{
	mVUinit(microVU1, 1);
	vu1Thread.Open();
	
	// ENABLED: Initialize asynchronous compilation system for VU1 to prevent stutters
	if (THREAD_VU1)
		g_microVU_AsyncCompiler.Initialize();
}

void recMicroVU0::Shutdown()
{
	mVUclose(microVU0);
}
void recMicroVU1::Shutdown()
{
	// ENABLED: Shutdown asynchronous compilation system first
	if (THREAD_VU1)
		g_microVU_AsyncCompiler.Shutdown();
	
	if (vu1Thread.IsOpen())
		vu1Thread.WaitVU();
	mVUclose(microVU1);
}

void recMicroVU0::Reset()
{
	mVUreset(microVU0, true);
}

void recMicroVU0::Step()
{
}

void recMicroVU1::Reset()
{
	vu1Thread.WaitVU();
	vu1Thread.Get_MTVUChanges();
	mVUreset(microVU1, true);
}

void recMicroVU0::SetStartPC(u32 startPC)
{
	VU0.start_pc = startPC;
}

void recMicroVU0::Execute(u32 cycles)
{
	VU0.flags &= ~VUFLAG_MFLAGSET;

	if (!(VU0.VI[REG_VPU_STAT].UL & 1))
		return;
	VU0.VI[REG_TPC].UL <<= 3;

	((mVUrecCall)microVU0.startFunct)(VU0.VI[REG_TPC].UL, cycles);
	VU0.VI[REG_TPC].UL >>= 3;
	if (microVU0.regs().flags & 0x4)
	{
		microVU0.regs().flags &= ~0x4;
		hwIntcIrq(6);
	}
}

void recMicroVU1::SetStartPC(u32 startPC)
{
	VU1.start_pc = startPC;
}

void recMicroVU1::Step()
{
}

void recMicroVU1::Execute(u32 cycles)
{
	if (!THREAD_VU1)
	{
		if (!(VU0.VI[REG_VPU_STAT].UL & 0x100))
			return;
	}
	VU1.VI[REG_TPC].UL <<= 3;
	((mVUrecCall)microVU1.startFunct)(VU1.VI[REG_TPC].UL, cycles);
	VU1.VI[REG_TPC].UL >>= 3;
	if (microVU1.regs().flags & 0x4 && !THREAD_VU1)
	{
		microVU1.regs().flags &= ~0x4;
		hwIntcIrq(7);
	}
}

void recMicroVU0::Clear(u32 addr, u32 size)
{
	mVUclear(microVU0, addr, size);
}
void recMicroVU1::Clear(u32 addr, u32 size)
{
	mVUclear(microVU1, addr, size);
}

void recMicroVU1::ResumeXGkick()
{
	if (!(VU0.VI[REG_VPU_STAT].UL & 0x100))
		return;
	((mVUrecCallXG)microVU1.startFunctXG)();
}

bool SaveStateBase::vuJITFreeze()
{
	if (IsSaving())
		vu1Thread.WaitVU();

	Freeze(microVU0.prog.lpState);
	Freeze(microVU1.prog.lpState);
	return IsOkay();
}

#if 0

#include <zlib.h>

void DumpVUState(u32 n, u32 pc)
{
	const VURegs& r = g_cpuRegistersPack.vuRegs[n];
	const microVU& mVU = (n == 0) ? microVU0 : microVU1;
	static FILE* fp = nullptr;
	static bool fp_opened = false;
	static u32 counter = 0;

	u32 first = pc >> 31;
	pc &= 0x7FFFFFFFu;
	if (first)
		counter++;

#if 0
	if (counter == 184639 && pc == 0x0D70)
		__debugbreak();
#endif

	if (counter < 0)
		return;

	if (!fp_opened)
	{
		fp = std::fopen("C:\\Dumps\\comp\\vulog.txt", "wb");
		fp_opened = true;
	}
	if (fp)
	{
		const microVU& m = (n == 0) ? microVU0 : microVU1;
		fprintf(fp, "%08d VU%u SPC:%04X xPC:%04X BRANCH:%04X VIBACKUP:%04X", counter, n, r.start_pc, pc, mVU.branch, mVU.VIbackup);
#if 1
		//fprintf(fp, " MEM:%08X", crc32(0, (Bytef*)r.Mem, (n == 0) ? VU0_MEMSIZE : VU1_MEMSIZE));
		fprintf(fp, " MAC %08X %08X %08X %08X [%08X %08X %08X %08X]", r.micro_macflags[3], r.micro_macflags[2], r.micro_macflags[1], r.micro_macflags[0], m.macFlag[3], m.macFlag[2], m.macFlag[1], m.macFlag[0]);
		fprintf(fp, " CLIP %08X %08X %08X %08X [%08X %08X %08X %08X]", r.micro_clipflags[3], r.micro_clipflags[2], r.micro_clipflags[1], r.micro_clipflags[0], m.clipFlag[3], m.clipFlag[2], m.clipFlag[1], m.clipFlag[0]);
		fprintf(fp, " STATUS %08X %08X %08X %08X [%08X %08X %08X %08X]", r.micro_statusflags[3], r.micro_statusflags[2], r.micro_statusflags[1], r.micro_statusflags[0], m.statFlag[3], m.statFlag[2], m.statFlag[1], m.statFlag[0]);

		for (u32 i = 0; i < 32; i++)
		{
			const VECTOR& v = r.VF[i];
			fprintf(fp, " VF%u: %08X%08X%08X%08X (%f,%f,%f,%f)", i, v.UL[3], v.UL[2], v.UL[1], v.UL[0], v.F[3], v.F[2], v.F[1], v.F[0]);
		}

		for (u32 i = 0; i < 32; i++)
		{
			const REG_VI& v = r.VI[i];
			fprintf(fp, " VI%u: %08X", i, v.UL);
		}

		fprintf(fp, " ACC: %08X%08X%08X%08X (%f,%f,%f,%f)", r.ACC.UL[3], r.ACC.UL[2], r.ACC.UL[1], r.ACC.UL[0],
			r.ACC.F[3], r.ACC.F[2], r.ACC.F[1], r.ACC.F[0]);
		fprintf(fp, " Q: %08X (%f)", r.q.UL, r.q.F);
		fprintf(fp, " P: %08X (%f)\n", r.p.UL, r.p.F);
#else
		fprintf(fp, " REG:%08X\n", crc32(0, (Bytef*)&r, offsetof(VURegs, idx)));
#endif
		//fflush(fp);
	}
}

#endif

//------------------------------------------------------------------
// COP2 Interface Functions
//------------------------------------------------------------------

void mVUFreeCOP2XMMreg(int hostreg)
{
	microVU0.regAlloc->clearRegCOP2(hostreg);
}

void mVUFreeCOP2GPR(int hostreg)
{
	microVU0.regAlloc->clearGPRCOP2(hostreg);
}

bool mVUIsReservedCOP2(int hostreg)
{
	// gprF1 through 3 is not correctly used in COP2 mode.
	return (hostreg == gprT1.GetCode() || hostreg == gprT2.GetCode() || hostreg == gprF0.GetCode());
}

//------------------------------------------------------------------
// COP2 Dispatch Tables (moved from microVU_Macro.inl to avoid duplicate symbols)
//------------------------------------------------------------------

// Forward declarations for functions defined in microVU_Macro.inl
void rec_C2UNK();
void recQMFC2();
void recCFC2();
void recQMTC2();
void recCTC2();
void recCOP2_BC2();
void recCOP2_SPEC1();
void recBC2F();
void recBC2T();
void recBC2FL();
void recBC2TL();

// Recompilation tables
void (*recCOP2t[32])() = {
	rec_C2UNK,     recQMFC2,      recCFC2,       rec_C2UNK,     rec_C2UNK,     recQMTC2,      recCTC2,       rec_C2UNK,
	recCOP2_BC2,   rec_C2UNK,     rec_C2UNK,     rec_C2UNK,     rec_C2UNK,     rec_C2UNK,     rec_C2UNK,     rec_C2UNK,
	recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1,
	recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1, recCOP2_SPEC1,
};

void (*recCOP2_BC2t[32])() = {
	recBC2F,   recBC2T,   recBC2FL,  recBC2TL,  rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK,
	rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK,
	rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK,
	rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK, rec_C2UNK,
};

// Forward declarations for VU instruction recompiler functions
void recVADDx(); void recVADDy(); void recVADDz(); void recVADDw(); void recVSUBx(); void recVSUBy(); void recVSUBz(); void recVSUBw();
void recVMADDx(); void recVMADDy(); void recVMADDz(); void recVMADDw(); void recVMSUBx(); void recVMSUBy(); void recVMSUBz(); void recVMSUBw();
void recVMAXx(); void recVMAXy(); void recVMAXz(); void recVMAXw(); void recVMINIx(); void recVMINIy(); void recVMINIz(); void recVMINIw();
void recVMULx(); void recVMULy(); void recVMULz(); void recVMULw(); void recVMULq(); void recVMAXi(); void recVMULi(); void recVMINIi();
void recVADDq(); void recVMADDq(); void recVADDi(); void recVMADDi(); void recVSUBq(); void recVMSUBq(); void recVSUBi(); void recVMSUBi();
void recVADD(); void recVMADD(); void recVMUL(); void recVMAX(); void recVSUB(); void recVMSUB(); void recVOPMSUB(); void recVMINI();
void recVIADD(); void recVISUB(); void recVIADDI(); void recVIAND(); void recVIOR();
void recVCALLMS(); void recVCALLMSR(); void recCOP2_SPEC2();

void (*recCOP2SPECIAL1t[64])() = {
	recVADDx,   recVADDy,   recVADDz,  recVADDw,  recVSUBx,      recVSUBy,      recVSUBz,      recVSUBw,
	recVMADDx,  recVMADDy,  recVMADDz, recVMADDw, recVMSUBx,     recVMSUBy,     recVMSUBz,     recVMSUBw,
	recVMAXx,   recVMAXy,   recVMAXz,  recVMAXw,  recVMINIx,     recVMINIy,     recVMINIz,     recVMINIw,
	recVMULx,   recVMULy,   recVMULz,  recVMULw,  recVMULq,      recVMAXi,      recVMULi,      recVMINIi,
	recVADDq,   recVMADDq,  recVADDi,  recVMADDi, recVSUBq,      recVMSUBq,     recVSUBi,      recVMSUBi,
	recVADD,    recVMADD,   recVMUL,   recVMAX,   recVSUB,       recVMSUB,      recVOPMSUB,    recVMINI,
	recVIADD,   recVISUB,   recVIADDI, rec_C2UNK, recVIAND,      recVIOR,       rec_C2UNK,     rec_C2UNK,
	recVCALLMS, recVCALLMSR,rec_C2UNK, rec_C2UNK, recCOP2_SPEC2, recCOP2_SPEC2, recCOP2_SPEC2, recCOP2_SPEC2,
};

// Forward declarations for VU SPECIAL2 instruction recompiler functions  
void recVADDAx(); void recVADDAy(); void recVADDAz(); void recVADDAw(); void recVSUBAx(); void recVSUBAy(); void recVSUBAz(); void recVSUBAw();
void recVMADDAx(); void recVMADDAy(); void recVMADDAz(); void recVMADDAw(); void recVMSUBAx(); void recVMSUBAy(); void recVMSUBAz(); void recVMSUBAw();
void recVITOF0(); void recVITOF4(); void recVITOF12(); void recVITOF15(); void recVFTOI0(); void recVFTOI4(); void recVFTOI12(); void recVFTOI15();
void recVMULAx(); void recVMULAy(); void recVMULAz(); void recVMULAw(); void recVMULAq(); void recVABS(); void recVMULAi(); void recVCLIP();
void recVADDAq(); void recVMADDAq(); void recVADDAi(); void recVMADDAi(); void recVSUBAq(); void recVMSUBAq(); void recVSUBAi(); void recVMSUBAi();
void recVADDA(); void recVMADDA(); void recVMULA(); void recVSUBA(); void recVMSUBA(); void recVOPMULA(); void recVNOP();
void recVMOVE(); void recVMR32(); void recVLQI(); void recVSQI(); void recVLQD(); void recVSQD();
void recVDIV(); void recVSQRT(); void recVRSQRT(); void recVWAITQ(); void recVMTIR(); void recVMFIR(); void recVILWR(); void recVISWR();
void recVRNEXT(); void recVRGET(); void recVRINIT(); void recVRXOR();

void (*recCOP2SPECIAL2t[128])() = {
	recVADDAx,  recVADDAy, recVADDAz,  recVADDAw,  recVSUBAx,  recVSUBAy,  recVSUBAz,  recVSUBAw,
	recVMADDAx,recVMADDAy, recVMADDAz, recVMADDAw, recVMSUBAx, recVMSUBAy, recVMSUBAz, recVMSUBAw,
	recVITOF0,  recVITOF4, recVITOF12, recVITOF15, recVFTOI0,  recVFTOI4,  recVFTOI12, recVFTOI15,
	recVMULAx,  recVMULAy, recVMULAz,  recVMULAw,  recVMULAq,  recVABS,    recVMULAi,  recVCLIP,
	recVADDAq,  recVMADDAq,recVADDAi,  recVMADDAi, recVSUBAq,  recVMSUBAq, recVSUBAi,  recVMSUBAi,
	recVADDA,   recVMADDA, recVMULA,   rec_C2UNK,  recVSUBA,   recVMSUBA,  recVOPMULA, recVNOP,
	recVMOVE,   recVMR32,  rec_C2UNK,  rec_C2UNK,  recVLQI,    recVSQI,    recVLQD,    recVSQD,
	recVDIV,    recVSQRT,  recVRSQRT,  recVWAITQ,  recVMTIR,   recVMFIR,   recVILWR,   recVISWR,
	recVRNEXT,  recVRGET,  recVRINIT,  recVRXOR,   rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
	rec_C2UNK,  rec_C2UNK, rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,  rec_C2UNK,
};

//------------------------------------------------------------------
// COP2 Dispatch Functions
//------------------------------------------------------------------

void recCOP2_BC2() { recCOP2_BC2t[_Rt_](); }

void recCOP2_SPEC1()
{
	if (g_pCurInstInfo->info & (EEINST_COP2_SYNC_VU0 | EEINST_COP2_FINISH_VU0))
		mVUFinishVU0();

	recCOP2SPECIAL1t[_Funct_]();
}

void recCOP2_SPEC2() { recCOP2SPECIAL2t[(cpuRegs.code & 3) | ((cpuRegs.code >> 4) & 0x7c)](); }

//------------------------------------------------------------------
// R5900::Dynarec::OpcodeImpl Namespace Functions
//------------------------------------------------------------------

// Forward declarations for functions defined in microVU_Macro.inl
void mVUSyncVU0();
void mVUFinishVU0();

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {

void recCOP2() { recCOP2t[_Rs_](); }

#if defined(LOADSTORE_RECOMPILE) && defined(CP2_RECOMPILE)

void recLQC2()
{
	if (g_pCurInstInfo->info & EEINST_COP2_SYNC_VU0)
		mVUSyncVU0();
	else if (g_pCurInstInfo->info & EEINST_COP2_FINISH_VU0)
		mVUFinishVU0();

	vtlb_ReadRegAllocCallback alloc_cb = nullptr;
	if (_Rt_)
	{
		// init regalloc after flush
		alloc_cb = []() { return _allocVFtoXMMreg(_Rt_, MODE_WRITE); };
	}

	int xmmreg;
	if (GPR_IS_CONST1(_Rs_))
	{
		const u32 addr = (g_cpuConstRegs[_Rs_].UL[0] + _Imm_) & ~0xFu;
		xmmreg = vtlb_DynGenReadQuad_Const(128, addr, alloc_cb);
	}
	else
	{
		_eeMoveGPRtoR(a64::WRegister(ECX), _Rs_);
		if (_Imm_ != 0) {
			armAsm->Add(ECX, ECX, _Imm_);
		}
		armAsm->And(ECX, ECX, ~0xF);

		xmmreg = vtlb_DynGenReadQuad(128, ECX.GetCode(), alloc_cb);
	}

	// toss away if loading to vf00
	if (!_Rt_)
		_freeXMMreg(xmmreg);

	EE::Profiler.EmitOp(eeOpcode::LQC2);
}

void recSQC2()
{
	if (g_pCurInstInfo->info & EEINST_COP2_SYNC_VU0)
		mVUSyncVU0();
	else if (g_pCurInstInfo->info & EEINST_COP2_FINISH_VU0)
		mVUFinishVU0();

	// vf00 has to be special cased here, because of the microvu temps...
	const int ftreg = _Rt_ ? _allocVFtoXMMreg(_Rt_, MODE_READ) : _allocTempXMMreg(XMMT_FPS);
	if (!_Rt_) {
		armAsm->Ldr(a64::QRegister(ftreg).Q(), PTR_CPU(vuRegs[0].VF[0].F));
	}

	if (GPR_IS_CONST1(_Rs_))
	{
		const u32 addr = (g_cpuConstRegs[_Rs_].UL[0] + _Imm_) & ~0xFu;
		vtlb_DynGenWrite_Const(128, true, addr, ftreg);
	}
	else
	{
		_eeMoveGPRtoR(a64::WRegister(ECX), _Rs_);
		if (_Imm_ != 0) {
			armAsm->Add(ECX, ECX, _Imm_);
		}
		armAsm->And(ECX, ECX, ~0xF);

		vtlb_DynGenWrite(128, true, ECX.GetCode(), ftreg);
	}

	if (!_Rt_)
		_freeXMMreg(ftreg);

	EE::Profiler.EmitOp(eeOpcode::SQC2);
}

#endif

} // namespace OpcodeImpl
} // namespace Dynarec
} // namespace R5900

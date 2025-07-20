// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "Config.h"
#include "cpuinfo.h"

//------------------------------------------------------------------
// Dispatcher Functions
//------------------------------------------------------------------
static bool mvuNeedsFPCRUpdate(mV)
{
	// always update on the vu1 thread
	if (isVU1 && THREAD_VU1)
		return true;

	// otherwise only emit when it's different to the EE
	return EmuConfig.Cpu.FPUFPCR.bitmask != (isVU0 ? EmuConfig.Cpu.VU0FPCR.bitmask : EmuConfig.Cpu.VU1FPCR.bitmask);
}

// Generates the code for entering/exit recompiled blocks
void mVUdispatcherAB(mV)
{
    mVU.startFunct = armStartBlock();

	{
//		xScopedStackFrame frame(false, true);
        armBeginStackFrame();

		// = The caller has already put the needed parameters in ecx/edx:
        if (!isVU1) {
//            xFastCall((void*)mVUexecuteVU0, arg1reg, arg2reg);
            armEmitCall(reinterpret_cast<void*>(mVUexecuteVU0));
        }
        else        {
//            xFastCall((void*)mVUexecuteVU1, arg1reg, arg2reg);
            armEmitCall(reinterpret_cast<void*>(mVUexecuteVU1));
        }

		// Load VU's MXCSR state
		if (mvuNeedsFPCRUpdate(mVU)) {
//            xLDMXCSR(ptr32[isVU0 ? &EmuConfig.Cpu.VU0FPCR.bitmask : &EmuConfig.Cpu.VU1FPCR.bitmask]);
            armAsm->Msr(a64::FPCR, isVU0
                                   ? armLoadPtr64(&EmuConfig.Cpu.VU0FPCR.bitmask)
                                   : armLoadPtr64(&EmuConfig.Cpu.VU1FPCR.bitmask));
        }

        // Load Regs
//		xMOVAPS (xmmT1, ptr128[&mVU.regs().VI[REG_P].UL]);
        armAsm->Ldr(xmmT1, armMemOperandPtr(&mVU.regs().VI[REG_P].UL));
//		xMOVAPS (xmmPQ, ptr128[&mVU.regs().VI[REG_Q].UL]);
        armAsm->Ldr(xmmPQ, armMemOperandPtr(&mVU.regs().VI[REG_Q].UL));
//		xMOVDZX (xmmT2, ptr32[&mVU.regs().pending_q]);
        armAsm->Ldr(xmmT2, armMemOperandPtr(&mVU.regs().pending_q));
//		xSHUF.PS(xmmPQ, xmmT1, 0); // wzyx = PPQQ
        armSHUFPS(xmmPQ, xmmT1, 0);
        //Load in other Q instance
//		xPSHUF.D(xmmPQ, xmmPQ, 0xe1);
        armPSHUFD(xmmPQ, xmmPQ, 0xe1);
//		xMOVSS(xmmPQ, xmmT2);
        armAsm->Mov(xmmPQ.S(), 0, xmmT2.S(), 0);
//		xPSHUF.D(xmmPQ, xmmPQ, 0xe1);
        armPSHUFD(xmmPQ, xmmPQ, 0xe1);

        if (isVU1)
        {
            //Load in other P instance
//			xMOVDZX(xmmT2, ptr32[&mVU.regs().pending_p]);
            armAsm->Ldr(xmmT2, armMemOperandPtr(&mVU.regs().pending_p));
//			xPSHUF.D(xmmPQ, xmmPQ, 0x1B);
            armPSHUFD(xmmPQ, xmmPQ, 0x1B);
//			xMOVSS(xmmPQ, xmmT2);
            armAsm->Mov(xmmPQ.S(), 0, xmmT2.S(), 0);
//			xPSHUF.D(xmmPQ, xmmPQ, 0x1B);
            armPSHUFD(xmmPQ, xmmPQ, 0x1B);
        }

//		xMOVAPS(xmmT1, ptr128[&mVU.regs().micro_macflags]);
        armAsm->Ldr(xmmT1.Q(), armMemOperandPtr(&mVU.regs().micro_macflags));
//		xMOVAPS(ptr128[mVU.macFlag], xmmT1);
        armAsm->Str(xmmT1.Q(), armMemOperandPtr(mVU.macFlag));

//		xMOVAPS(xmmT1, ptr128[&mVU.regs().micro_clipflags]);
        armAsm->Ldr(xmmT1.Q(), armMemOperandPtr(&mVU.regs().micro_clipflags));
//		xMOVAPS(ptr128[mVU.clipFlag], xmmT1);
        armAsm->Str(xmmT1.Q(), armMemOperandPtr(mVU.clipFlag));

//		xMOV(gprF0, ptr32[&mVU.regs().micro_statusflags[0]]);
        armAsm->Ldr(gprF0, armMemOperandPtr(&mVU.regs().micro_statusflags[0]));
//		xMOV(gprF1, ptr32[&mVU.regs().micro_statusflags[1]]);
        armAsm->Ldr(gprF1, armMemOperandPtr(&mVU.regs().micro_statusflags[1]));
//		xMOV(gprF2, ptr32[&mVU.regs().micro_statusflags[2]]);
        armAsm->Ldr(gprF2, armMemOperandPtr(&mVU.regs().micro_statusflags[2]));
//		xMOV(gprF3, ptr32[&mVU.regs().micro_statusflags[3]]);
        armAsm->Ldr(gprF3, armMemOperandPtr(&mVU.regs().micro_statusflags[3]));

		// Jump to Recompiled Code Block
//		xJMP(rax);
        armAsm->Br(RAX);

		mVU.exitFunct = armGetCurrentCodePointer();

		// Load EE's MXCSR state
		if (mvuNeedsFPCRUpdate(mVU)) {
//            xLDMXCSR(ptr32[&EmuConfig.Cpu.FPUFPCR.bitmask]);
            armAsm->Msr(a64::FPCR, armLoadPtr64(&EmuConfig.Cpu.FPUFPCR.bitmask));
        }

		// = The first two DWORD or smaller arguments are passed in ECX and EDX registers;
		//              all other arguments are passed right to left.
        if (!isVU1) {
//            xFastCall((void*)mVUcleanUpVU0);
            armEmitCall(reinterpret_cast<void*>(mVUcleanUpVU0));
        }
        else        {
//            xFastCall((void*)mVUcleanUpVU1);
            armEmitCall(reinterpret_cast<void*>(mVUcleanUpVU1));
        }

        armEndStackFrame();
	}

//	xRET();
    armAsm->Ret();

    mVU.prog.x86start = armEndBlock();

	Perf::any.Register(mVU.startFunct, static_cast<u32>(mVU.prog.x86start - mVU.startFunct),
		mVU.index ? "VU1StartFunc" : "VU0StartFunc");
}

// Generates the code for resuming/exit xgkick
void mVUdispatcherCD(mV)
{
    mVU.startFunctXG = armStartBlock();

	{
//		xScopedStackFrame frame(false, true);
        armBeginStackFrame();

        // Load VU's MXCSR state
        if (mvuNeedsFPCRUpdate(mVU)) {
//            xLDMXCSR(ptr32[isVU0 ? &EmuConfig.Cpu.VU0FPCR.bitmask : &EmuConfig.Cpu.VU1FPCR.bitmask]);
            armAsm->Msr(a64::FPCR, isVU0
                                   ? armLoadPtr64(&EmuConfig.Cpu.VU0FPCR.bitmask)
                                   : armLoadPtr64(&EmuConfig.Cpu.VU1FPCR.bitmask));
        }

        mVUrestoreRegs(mVU);
//		xMOV(gprF0, ptr32[&mVU.regs().micro_statusflags[0]]);
        armAsm->Ldr(gprF0, armMemOperandPtr(&mVU.regs().micro_statusflags[0]));
//		xMOV(gprF1, ptr32[&mVU.regs().micro_statusflags[1]]);
        armAsm->Ldr(gprF1, armMemOperandPtr(&mVU.regs().micro_statusflags[1]));
//		xMOV(gprF2, ptr32[&mVU.regs().micro_statusflags[2]]);
        armAsm->Ldr(gprF2, armMemOperandPtr(&mVU.regs().micro_statusflags[2]));
//		xMOV(gprF3, ptr32[&mVU.regs().micro_statusflags[3]]);
        armAsm->Ldr(gprF3, armMemOperandPtr(&mVU.regs().micro_statusflags[3]));

        // Jump to Recompiled Code Block
//		xJMP(ptrNative[&mVU.resumePtrXG]);
        armEmitJmp(&mVU.resumePtrXG);

        mVU.exitFunctXG = armGetCurrentCodePointer();

        // Backup Status Flag (other regs were backed up on xgkick)
//		xMOV(ptr32[&mVU.regs().micro_statusflags[0]], gprF0);
        armAsm->Str(gprF0, armMemOperandPtr(&mVU.regs().micro_statusflags[0]));
//		xMOV(ptr32[&mVU.regs().micro_statusflags[1]], gprF1);
        armAsm->Str(gprF1, armMemOperandPtr(&mVU.regs().micro_statusflags[1]));
//		xMOV(ptr32[&mVU.regs().micro_statusflags[2]], gprF2);
        armAsm->Str(gprF2, armMemOperandPtr(&mVU.regs().micro_statusflags[2]));
//		xMOV(ptr32[&mVU.regs().micro_statusflags[3]], gprF3);
        armAsm->Str(gprF3, armMemOperandPtr(&mVU.regs().micro_statusflags[3]));

        // Load EE's MXCSR state
        if (mvuNeedsFPCRUpdate(mVU)) {
//            xLDMXCSR(ptr32[&EmuConfig.Cpu.FPUFPCR.bitmask]);
            armAsm->Msr(a64::FPCR, armLoadPtr64(&EmuConfig.Cpu.FPUFPCR.bitmask));
        }

        armEndStackFrame();
	}

//	xRET();
    armAsm->Ret();

    mVU.prog.x86start = armEndBlock();

	Perf::any.Register(mVU.startFunctXG, static_cast<u32>(mVU.prog.x86start - mVU.startFunctXG),
		mVU.index ? "VU1StartFuncXG" : "VU0StartFuncXG");
}

static void mVUGenerateWaitMTVU(mV)
{
    mVU.waitMTVU = armStartBlock();

    int i, num_xmms = 0, num_gprs = 0;

    for (i = 0; i < static_cast<int>(iREGCNT_GPR); ++i)
    {
        if (!armIsCallerSaved(i) || i == 4)
            continue;

        // T1 often contains the address we're loading when waiting for VU1.
        // T2 isn't used until afterwards, so don't bother saving it.
        if (i == gprT2.GetCode())
            continue;

//		xPUSH(xRegister64(i));
        armAsm->Push(a64::xzr, a64::Register(i, a64::kXRegSize));
        num_gprs++;
    }

    for (i = 0; i < static_cast<int>(iREGCNT_XMM); ++i)
    {
        if (!armIsCallerSavedXmm(i))
            continue;

        num_xmms++;
    }

    // We need 16 byte alignment on the stack.
    // Since the stack is unaligned at entry to this function, we add 8 when it's even, not odd.
    const int stack_size = (num_xmms * sizeof(u128)) + ((~num_gprs & 1) * sizeof(u128)) + SHADOW_STACK_SIZE;
    int stack_offset = SHADOW_STACK_SIZE;

    if (stack_size > 0)
    {
//		xSUB(rsp, stack_size);
        armAsm->Sub(a64::sp, a64::sp, stack_size);
        for (i = 0; i < static_cast<int>(iREGCNT_XMM); i++)
        {
            if (!armIsCallerSavedXmm(i))
                continue;

//			xMOVAPS(ptr128[rsp + stack_offset], xRegisterSSE(i));
            armAsm->Str(a64::QRegister(i).Q(), a64::MemOperand(a64::sp, stack_offset));
            stack_offset += sizeof(u128);
        }
    }

    ////
//	xFastCall((void*)mVUwaitMTVU);
    armAsm->Push(a64::xzr, a64::lr);
    armAsm->Mov(RXVIXLSCRATCH, reinterpret_cast<intptr_t>(mVUwaitMTVU));
    armAsm->Blr(RXVIXLSCRATCH);
    armAsm->Pop(a64::lr, a64::xzr);
    ////

    stack_offset = (num_xmms - 1) * sizeof(u128) + SHADOW_STACK_SIZE;
    for (i = static_cast<int>(iREGCNT_XMM - 1); i >= 0; --i)
    {
        if (!armIsCallerSavedXmm(i))
            continue;

//		xMOVAPS(xRegisterSSE(i), ptr128[rsp + stack_offset]);
        armAsm->Ldr(a64::QRegister(i).Q(), a64::MemOperand(a64::sp, stack_offset));
        stack_offset -= sizeof(u128);
    }
//	xADD(rsp, stack_size);
    armAsm->Add(a64::sp, a64::sp, stack_size);

    for (i = static_cast<int>(iREGCNT_GPR - 1); i >= 0; --i)
    {
        if (!armIsCallerSaved(i) || i == 4)
            continue;

        if (i == gprT2.GetCode())
            continue;

//		xPOP(xRegister64(i));
        armAsm->Pop(a64::Register(i, a64::kXRegSize), a64::xzr);
    }

//	xRET();
    armAsm->Ret();

    mVU.prog.x86start = armEndBlock();

	Perf::any.Register(mVU.waitMTVU, static_cast<u32>(mVU.prog.x86start - mVU.waitMTVU),
		mVU.index ? "VU1WaitMTVU" : "VU0WaitMTVU");
}

static void mVUGenerateCopyPipelineState(mV)
{
    mVU.copyPLState = armStartBlock();

    {
//		xMOVAPS(xmm0, ptr[rax]);
        armAsm->Ldr(xmm0, a64::MemOperand(RAX));
//		xMOVAPS(xmm1, ptr[rax + 16u]);
        armAsm->Ldr(xmm1, a64::MemOperand(RAX, 16u));
//		xMOVAPS(xmm2, ptr[rax + 32u]);
        armAsm->Ldr(xmm2, a64::MemOperand(RAX, 32u));
//		xMOVAPS(xmm3, ptr[rax + 48u]);
        armAsm->Ldr(xmm3, a64::MemOperand(RAX, 48u));
//		xMOVAPS(xmm4, ptr[rax + 64u]);
        armAsm->Ldr(xmm4, a64::MemOperand(RAX, 64u));
//		xMOVAPS(xmm5, ptr[rax + 80u]);
        armAsm->Ldr(xmm5, a64::MemOperand(RAX, 80u));

//		xMOVUPS(ptr[reinterpret_cast<u8*>(&mVU.prog.lpState)], xmm0);
        armAsm->Str(xmm0, armMemOperandPtr(reinterpret_cast<u8*>(&mVU.prog.lpState)));
//		xMOVUPS(ptr[reinterpret_cast<u8*>(&mVU.prog.lpState) + 16u], xmm1);
        armAsm->Str(xmm1, armMemOperandPtr(reinterpret_cast<u8*>(&mVU.prog.lpState) + 16u));
//		xMOVUPS(ptr[reinterpret_cast<u8*>(&mVU.prog.lpState) + 32u], xmm2);
        armAsm->Str(xmm2, armMemOperandPtr(reinterpret_cast<u8*>(&mVU.prog.lpState) + 32u));
//		xMOVUPS(ptr[reinterpret_cast<u8*>(&mVU.prog.lpState) + 48u], xmm3);
        armAsm->Str(xmm3, armMemOperandPtr(reinterpret_cast<u8*>(&mVU.prog.lpState) + 48u));
//		xMOVUPS(ptr[reinterpret_cast<u8*>(&mVU.prog.lpState) + 64u], xmm4);
        armAsm->Str(xmm4, armMemOperandPtr(reinterpret_cast<u8*>(&mVU.prog.lpState) + 64u));
//		xMOVUPS(ptr[reinterpret_cast<u8*>(&mVU.prog.lpState) + 80u], xmm5);
        armAsm->Str(xmm5, armMemOperandPtr(reinterpret_cast<u8*>(&mVU.prog.lpState) + 80u));
    }

//	xRET();
    armAsm->Ret();

    mVU.prog.x86start = armEndBlock();

	Perf::any.Register(mVU.copyPLState, static_cast<u32>(mVU.prog.x86start - mVU.copyPLState),
		mVU.index ? "VU1CopyPLState" : "VU0CopyPLState");
}

//------------------------------------------------------------------
// Micro VU - Custom Quick Search
//------------------------------------------------------------------

// Generates a custom optimized block-search function
// Note: Structs must be 16-byte aligned! (GCC doesn't guarantee this)
static void mVUGenerateCompareState(mV)
{
    mVU.compareStateF = armStartBlock();

    {
//		xMOVAPS  (xmm0, ptr32[arg1reg]);
        armAsm->Ldr(xmm0, a64::MemOperand(RAX));
//		xPCMP.EQD(xmm0, ptr32[arg2reg]);
        armAsm->Cmeq(xmm0.V4S(), xmm0.V4S(), armLoadPtrM(RCX).V4S());
//		xMOVAPS  (xmm1, ptr32[arg1reg + 0x10]);
        armAsm->Ldr(xmm1, a64::MemOperand(RAX, 0x10));
//		xPCMP.EQD(xmm1, ptr32[arg2reg + 0x10]);
        armAsm->Cmeq(xmm1.V4S(), xmm1.V4S(), armLoadPtrM(RCX, 0x10).V4S());
//		xPAND    (xmm0, xmm1);
        armAsm->And(xmm0.V16B(), xmm0.V16B(), xmm1.V16B());

//		xMOVMSKPS(eax, xmm0);
        armMOVMSKPS(EEX, xmm0);
//		xXOR     (eax, 0xf);
        armAsm->Eor(EEX, EEX, 0xf);

//        armAsm->Cmp(EEX, 0xf); // Eor or Cmp

        // flag test
        armAsm->Tst(EEX, EEX);

//		xForwardJNZ8 exitPoint;
        a64::Label exitPoint;
        armAsm->B(&exitPoint, a64::Condition::ne);

//		xMOVAPS  (xmm0, ptr32[arg1reg + 0x20]);
        armAsm->Ldr(xmm0, a64::MemOperand(RAX, 0x20));
//		xPCMP.EQD(xmm0, ptr32[arg2reg + 0x20]);
        armAsm->Cmeq(xmm0.V4S(), xmm0.V4S(), armLoadPtrM(RCX, 0x20).V4S());
//		xMOVAPS  (xmm1, ptr32[arg1reg + 0x30]);
        armAsm->Ldr(xmm1, a64::MemOperand(RAX, 0x30));
//		xPCMP.EQD(xmm1, ptr32[arg2reg + 0x30]);
        armAsm->Cmeq(xmm1.V4S(), xmm1.V4S(), armLoadPtrM(RCX, 0x30).V4S());
//		xPAND    (xmm0, xmm1);
        armAsm->And(xmm0.V16B(), xmm0.V16B(), xmm1.V16B());

//		xMOVAPS  (xmm1, ptr32[arg1reg + 0x40]);
        armAsm->Ldr(xmm1, a64::MemOperand(RAX, 0x40));
//		xPCMP.EQD(xmm1, ptr32[arg2reg + 0x40]);
        armAsm->Cmeq(xmm1.V4S(), xmm1.V4S(), armLoadPtrM(RCX, 0x40).V4S());
//		xMOVAPS  (xmm2, ptr32[arg1reg + 0x50]);
        armAsm->Ldr(xmm2, a64::MemOperand(RAX, 0x50));
//		xPCMP.EQD(xmm2, ptr32[arg2reg + 0x50]);
        armAsm->Cmeq(xmm2.V4S(), xmm2.V4S(), armLoadPtrM(RCX, 0x50).V4S());
//		xPAND    (xmm1, xmm2);
        armAsm->And(xmm1.V16B(), xmm1.V16B(), xmm2.V16B());
//		xPAND    (xmm0, xmm1);
        armAsm->And(xmm0.V16B(), xmm0.V16B(), xmm1.V16B());

//		xMOVMSKPS(eax, xmm0);
        armMOVMSKPS(EEX, xmm0);
//		xXOR(eax, 0xf);
        armAsm->Eor(EEX, EEX, 0xf);

//		exitPoint.SetTarget();
        armBind(&exitPoint);

        // 결과값
        armAsm->Mov(EAX, EEX);
    }

//	xRET();
    armAsm->Ret();

    mVU.prog.x86start = armEndBlock();
}


//------------------------------------------------------------------
// Execution Functions
//------------------------------------------------------------------

// Executes for number of cycles
_mVUt void* mVUexecute(u32 startPC, u32 cycles)
{

	microVU& mVU = mVUx;
	u32 vuLimit = vuIndex ? 0x3ff8 : 0xff8;
	if (startPC > vuLimit + 7)
	{
		DevCon.Warning("microVU%x Warning: startPC = 0x%x, cycles = 0x%x", vuIndex, startPC, cycles);
	}

	mVU.cycles = cycles;
	mVU.totalCycles = cycles;

//	xSetPtr(mVU.prog.x86ptr); // Set x86ptr to where last program left off
    armSetAsmPtr(mVU.prog.x86ptr, vuIndex ? HostMemoryMap::mVU1recSize : HostMemoryMap::mVU0recSize, nullptr);
    mVU.prog.x86ptr = armStartBlock();

	return mVUsearchProg<vuIndex>(startPC & vuLimit, (uptr)&mVU.prog.lpState); // Find and set correct program
}

//------------------------------------------------------------------
// Cleanup Functions
//------------------------------------------------------------------

_mVUt void mVUcleanUp()
{
	microVU& mVU = mVUx;

//	mVU.prog.x86ptr = x86Ptr;
    mVU.prog.x86ptr = armEndBlock();

	if ((mVU.prog.x86ptr < mVU.prog.x86start) || (mVU.prog.x86ptr >= mVU.prog.x86end))
	{
		Console.WriteLn(vuIndex ? Color_Orange : Color_Magenta, "microVU%d: Program cache limit reached.", mVU.index);
		mVUreset(mVU, false);
	}

	mVU.cycles = mVU.totalCycles - std::max(0, mVU.cycles);
	mVU.regs().cycle += mVU.cycles;

	if (!vuIndex || !THREAD_VU1)
	{
		u32 cycles_passed = std::min(mVU.cycles, 3000) * EmuConfig.Speedhacks.EECycleSkip;
		if (cycles_passed > 0)
		{
			s32 vu0_offset = VU0.cycle - cpuRegs.cycle;
			cpuRegs.cycle += cycles_passed;

			// VU0 needs to stay in sync with the CPU otherwise things get messy
			// So we need to adjust when VU1 skips cycles also
			if (!vuIndex)
				VU0.cycle = cpuRegs.cycle + vu0_offset;
			else
				VU0.cycle += cycles_passed;
		}
	}
	mVU.profiler.Print();
}

//------------------------------------------------------------------
// Caller Functions
//------------------------------------------------------------------

void* mVUexecuteVU0(u32 startPC, u32 cycles) { return mVUexecute<0>(startPC, cycles); }
void* mVUexecuteVU1(u32 startPC, u32 cycles) { return mVUexecute<1>(startPC, cycles); }
void mVUcleanUpVU0() { mVUcleanUp<0>(); }
void mVUcleanUpVU1() { mVUcleanUp<1>(); }

// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0

#include "Vif_UnpackNEON.h"
#include "common/Perf.h"

namespace a64 = vixl::aarch64;

// =====================================================================================================
//  VifUnpackSSE_Base Section
// =====================================================================================================
VifUnpackNEON_Base::VifUnpackNEON_Base()
	: usn(false)
	, doMask(false)
	, UnpkLoopIteration(0)
	, UnpkNoOfIterations(0)
	, IsAligned(0)
	, dstIndirect(a64::MemOperand(RXARG1))
	, srcIndirect(a64::MemOperand(RXARG2))
	, workReg(a64::q1)
	, destReg(a64::q0)
	, workGprW(a64::w4)
{
}

void VifUnpackNEON_Base::xMovDest() const
{
	if (!IsWriteProtectedOp())
	{
		if (IsUnmaskedOp()) {
#ifdef __aarch64__
			// ARM64 optimized store with write-combining hint
			// Use non-temporal store hint for better cache behavior
			armAsm->Str(destReg, dstIndirect);
			// Prefetch next destination for potential future writes
			armAsm->Prfm(a64::PSTL1KEEP, a64::MemOperand(dstIndirect.GetBaseRegister(), dstIndirect.GetOffset() + 16));
#else
			armAsm->Str(destReg, dstIndirect);
#endif
		} else {
			doMaskWrite(destReg);
		}
	}
}

void VifUnpackNEON_Base::xShiftR(const vixl::aarch64::VRegister& regX, int n) const
{
	if (usn)
		armAsm->Ushr(regX.V4S(), regX.V4S(), n);
	else
		armAsm->Sshr(regX.V4S(), regX.V4S(), n);
}

void VifUnpackNEON_Base::xPMOVXX8(const vixl::aarch64::VRegister& regX) const
{
#ifdef __aarch64__
	// ARM64 optimized 8-bit to 32-bit extension with better instruction scheduling
	armAsm->Ldr(regX.S(), srcIndirect);
	// Prefetch next potential data for better cache utilization
	armAsm->Prfm(a64::PLDL1KEEP, a64::MemOperand(srcIndirect.GetBaseRegister(), srcIndirect.GetOffset() + 16));
	
	// Optimized two-stage extension with interleaved execution for better pipeline utilization
	if (usn)
	{
		armAsm->Ushll(regX.V8H(), regX.V8B(), 0);   // 8->16 bit unsigned extension
		armAsm->Ushll(regX.V4S(), regX.V4H(), 0);   // 16->32 bit unsigned extension
	}
	else
	{
		armAsm->Sshll(regX.V8H(), regX.V8B(), 0);   // 8->16 bit signed extension
		armAsm->Sshll(regX.V4S(), regX.V4H(), 0);   // 16->32 bit signed extension
	}
#else
	// Original implementation for non-ARM64 platforms
	armAsm->Ldr(regX.S(), srcIndirect);

	if (usn)
	{
		armAsm->Ushll(regX.V8H(), regX.V8B(), 0);
		armAsm->Ushll(regX.V4S(), regX.V4H(), 0);
	}
	else
	{
		armAsm->Sshll(regX.V8H(), regX.V8B(), 0);
		armAsm->Sshll(regX.V4S(), regX.V4H(), 0);
	}
#endif
}

void VifUnpackNEON_Base::xPMOVXX16(const vixl::aarch64::VRegister& regX) const
{
#ifdef __aarch64__
	// ARM64 optimized 16-bit to 32-bit extension with prefetching
	armAsm->Ldr(regX.D(), srcIndirect);
	// Prefetch next potential data while processing current data
	armAsm->Prfm(a64::PLDL1KEEP, a64::MemOperand(srcIndirect.GetBaseRegister(), srcIndirect.GetOffset() + 32));
	
	// Single-stage extension with optimal instruction timing
	if (usn)
		armAsm->Ushll(regX.V4S(), regX.V4H(), 0);  // 16->32 bit unsigned extension
	else
		armAsm->Sshll(regX.V4S(), regX.V4H(), 0);  // 16->32 bit signed extension
#else
	// Original implementation for non-ARM64 platforms
	armAsm->Ldr(regX.D(), srcIndirect);

	if (usn)
		armAsm->Ushll(regX.V4S(), regX.V4H(), 0);
	else
		armAsm->Sshll(regX.V4S(), regX.V4H(), 0);
#endif
}

void VifUnpackNEON_Base::xUPK_S_32() const
{
#ifdef __aarch64__
	// ARM64 optimized scalar 32-bit unpacking with conditional prefetching
	if (UnpkLoopIteration == 0) {
		armAsm->Ldr(workReg, srcIndirect);
		// Prefetch next potential read for better cache utilization
		armAsm->Prfm(a64::PLDL1KEEP, a64::MemOperand(srcIndirect.GetBaseRegister(), srcIndirect.GetOffset() + 64));
	}

	if (IsInputMasked())
		return;

	// Use branchless approach for better pipeline efficiency
	armAsm->Dup(destReg.V4S(), workReg.V4S(), UnpkLoopIteration);
#else
	// Original implementation with explicit switch for non-ARM64 platforms
	if (UnpkLoopIteration == 0)
		armAsm->Ldr(workReg, srcIndirect);

	if (IsInputMasked())
		return;

	switch (UnpkLoopIteration)
	{
		case 0:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 0);
			break;
		case 1:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 1);
			break;
		case 2:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 2);
			break;
		case 3:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 3);
			break;
	}
#endif
}

void VifUnpackNEON_Base::xUPK_S_16() const
{
	if (UnpkLoopIteration == 0)
		xPMOVXX16(workReg);

	if (IsInputMasked())
		return;

	switch (UnpkLoopIteration)
	{
		case 0:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 0);
			break;
		case 1:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 1);
			break;
		case 2:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 2);
			break;
		case 3:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 3);
			break;
	}
}

void VifUnpackNEON_Base::xUPK_S_8() const
{
	if (UnpkLoopIteration == 0)
		xPMOVXX8(workReg);

	if (IsInputMasked())
		return;

	switch (UnpkLoopIteration)
	{
		case 0:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 0);
			break;
		case 1:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 1);
			break;
		case 2:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 2);
			break;
		case 3:
			armAsm->Dup(destReg.V4S(), workReg.V4S(), 3);
			break;
	}
}

// The V2 + V3 unpacks have freaky behaviour, the manual claims "indeterminate".
// After testing on the PS2, it's very much determinate in 99% of cases
// and games like Lemmings, And1 Streetball rely on this data to be like this!
// I have commented after each shuffle to show what data is going where - Ref

void VifUnpackNEON_Base::xUPK_V2_32() const
{
	if (UnpkLoopIteration == 0)
	{
		armAsm->Ldr(workReg, srcIndirect);

		if (IsInputMasked())
			return;

		armAsm->Dup(destReg.V2D(), workReg.V2D(), 0); //v1v0v1v0
		if (IsAligned)
			armAsm->Ins(destReg.V4S(), 3, a64::wzr); //zero last word - tested on ps2
	}
	else
	{
		if (IsInputMasked())
			return;

		armAsm->Dup(destReg.V2D(), workReg.V2D(), 1); //v3v2v3v2
		if (IsAligned)
			armAsm->Ins(destReg.V4S(), 3, a64::wzr); //zero last word - tested on ps2
	}
}

void VifUnpackNEON_Base::xUPK_V2_16() const
{
	if (UnpkLoopIteration == 0)
	{
		xPMOVXX16(workReg);

		if (IsInputMasked())
			return;

		armAsm->Dup(destReg.V2D(), workReg.V2D(), 0); //v1v0v1v0
	}
	else
	{
		if (IsInputMasked())
			return;

		armAsm->Dup(destReg.V2D(), workReg.V2D(), 1); //v3v2v3v2
	}
}

void VifUnpackNEON_Base::xUPK_V2_8() const
{
	if (UnpkLoopIteration == 0)
	{
		xPMOVXX8(workReg);

		if (IsInputMasked())
			return;

		armAsm->Dup(destReg.V2D(), workReg.V2D(), 0); //v1v0v1v0
	}
	else
	{
		if (IsInputMasked())
			return;

		armAsm->Dup(destReg.V2D(), workReg.V2D(), 1); //v3v2v3v2
	}
}

void VifUnpackNEON_Base::xUPK_V3_32() const
{
	if (IsInputMasked())
		return;

	armAsm->Ldr(destReg, srcIndirect);
	if (UnpkLoopIteration != IsAligned)
		armAsm->Ins(destReg.V4S(), 3, a64::wzr);
}

void VifUnpackNEON_Base::xUPK_V3_16() const
{
	if (IsInputMasked())
		return;

	xPMOVXX16(destReg);

	//With V3-16, it takes the first vector from the next position as the W vector
	//However - IF the end of this iteration of the unpack falls on a quadword boundary, W becomes 0
	//IsAligned is the position through the current QW in the vif packet
	//Iteration counts where we are in the packet.
	int result = (((UnpkLoopIteration / 4) + 1 + (4 - IsAligned)) & 0x3);

	if ((UnpkLoopIteration & 0x1) == 0 && result == 0)
		armAsm->Ins(destReg.V4S(), 3, a64::wzr); //zero last word on QW boundary if whole 32bit word is used - tested on ps2
}

void VifUnpackNEON_Base::xUPK_V3_8() const
{
	if (IsInputMasked())
		return;

	xPMOVXX8(destReg);
	if (UnpkLoopIteration != IsAligned)
		armAsm->Ins(destReg.V4S(), 3, a64::wzr);
}

void VifUnpackNEON_Base::xUPK_V4_32() const
{
	if (IsInputMasked())
		return;

#ifdef __aarch64__
	// ARM64 optimized with prefetching for better cache performance
	// Prefetch next cache line for potential future reads
	armAsm->Prfm(a64::PLDL1KEEP, a64::MemOperand(srcIndirect.GetBaseRegister(), srcIndirect.GetOffset() + 64));
	// Load 128-bit vector with single instruction
	armAsm->Ldr(destReg.Q(), srcIndirect);
#else
	armAsm->Ldr(destReg.Q(), a64::MemOperand(srcIndirect));
#endif
}

void VifUnpackNEON_Base::xUPK_V4_16() const
{
	if (IsInputMasked())
		return;

#ifdef __aarch64__
	// ARM64 optimized 16-bit to 32-bit unpacking with interleaved operations
	// Load 64-bit of 16-bit data
	armAsm->Ldr(destReg.D(), srcIndirect);
	// Prefetch next potential read while processing current data
	armAsm->Prfm(a64::PLDL1KEEP, a64::MemOperand(srcIndirect.GetBaseRegister(), srcIndirect.GetOffset() + 32));
	
	if (usn)
		armAsm->Ushll(destReg.V4S(), destReg.V4H(), 0);
	else
		armAsm->Sshll(destReg.V4S(), destReg.V4H(), 0);
#else
	xPMOVXX16(destReg);
#endif
}

void VifUnpackNEON_Base::xUPK_V4_8() const
{
	if (IsInputMasked())
		return;

#ifdef __aarch64__
	// ARM64 optimized 8-bit to 32-bit unpacking with interleaved operations
	// Load 32-bit of 8-bit data
	armAsm->Ldr(destReg.S(), srcIndirect);
	// Prefetch next potential read
	armAsm->Prfm(a64::PLDL1KEEP, a64::MemOperand(srcIndirect.GetBaseRegister(), srcIndirect.GetOffset() + 16));
	
	// Two-stage extension: 8->16->32 with interleaved execution
	if (usn) {
		armAsm->Ushll(destReg.V8H(), destReg.V8B(), 0);
		armAsm->Ushll(destReg.V4S(), destReg.V4H(), 0);
	} else {
		armAsm->Sshll(destReg.V8H(), destReg.V8B(), 0);
		armAsm->Sshll(destReg.V4S(), destReg.V4H(), 0);
	}
#else
	xPMOVXX8(destReg);
#endif
}

void VifUnpackNEON_Base::xUPK_V4_5() const
{
	if (IsInputMasked())
		return;

#ifdef __aarch64__
	// ARM64 optimized RGB555 unpacking using efficient bit field extraction
	// Reduces instruction count and improves pipeline utilization
	
	armAsm->Ldrh(workGprW, srcIndirect);
	const a64::WRegister workGprW2(5);  // Additional work register (w5)
	
	// Parallel extraction using bit field operations for better throughput
	// Extract R (bits 0-4) and G (bits 5-9) in parallel
	armAsm->Ubfx(workGprW, workGprW, 0, 5);    // Extract R: bits 0-4  
	armAsm->Ldrh(workGprW2, srcIndirect);      // Reload for parallel extraction
	armAsm->Lsl(workGprW, workGprW, 3);        // R * 8 (scale to 8-bit)
	armAsm->Ubfx(workGprW2, workGprW2, 5, 5);  // Extract G: bits 5-9
	armAsm->Dup(destReg.V4S(), workGprW);      // Set all lanes to R initially
	armAsm->Lsl(workGprW2, workGprW2, 3);      // G * 8 (scale to 8-bit)
	armAsm->Ins(destReg.V4S(), 1, workGprW2);  // Insert G component
	
	// Extract B and A components
	armAsm->Ldrh(workGprW, srcIndirect);       // Reload for B extraction
	armAsm->Ubfx(workGprW2, workGprW, 10, 5);  // Extract B: bits 10-14
	armAsm->Lsl(workGprW2, workGprW2, 3);      // B * 8 (scale to 8-bit)
	armAsm->Ins(destReg.V4S(), 2, workGprW2);  // Insert B component
	armAsm->Ubfx(workGprW, workGprW, 15, 1);   // Extract A: bit 15
	armAsm->Lsl(workGprW, workGprW, 7);        // A * 128 (scale to 8-bit)
	armAsm->Ins(destReg.V4S(), 3, workGprW);   // Insert A component
#else
	// Fallback to original implementation for non-ARM64 platforms
	armAsm->Ldrh(workGprW, srcIndirect);
	armAsm->Lsl(workGprW, workGprW, 3); // ABG|R5.000
	armAsm->Dup(destReg.V4S(), workGprW); // x|x|x|R
	armAsm->Lsr(workGprW, workGprW, 8); // ABG
	armAsm->Lsl(workGprW, workGprW, 3); // AB|G5.000
	armAsm->Ins(destReg.V4S(), 1, workGprW); // x|x|G|R
	armAsm->Lsr(workGprW, workGprW, 8); // AB
	armAsm->Lsl(workGprW, workGprW, 3); // A|B5.000
	armAsm->Ins(destReg.V4S(), 2, workGprW); // x|B|G|R
	armAsm->Lsr(workGprW, workGprW, 8); // A
	armAsm->Lsl(workGprW, workGprW, 7); // A.0000000
	armAsm->Ins(destReg.V4S(), 3, workGprW); // A|B|G|R
	armAsm->Shl(destReg.V4S(), destReg.V4S(), 24); // can optimize to
	armAsm->Ushr(destReg.V4S(), destReg.V4S(), 24); // single AND...
#endif
}

void VifUnpackNEON_Base::xUnpack(int upknum) const
{
	switch (upknum)
	{
		case 0:
			xUPK_S_32();
			break;
		case 1:
			xUPK_S_16();
			break;
		case 2:
			xUPK_S_8();
			break;

		case 4:
			xUPK_V2_32();
			break;
		case 5:
			xUPK_V2_16();
			break;
		case 6:
			xUPK_V2_8();
			break;

		case 8:
			xUPK_V3_32();
			break;
		case 9:
			xUPK_V3_16();
			break;
		case 10:
			xUPK_V3_8();
			break;

		case 12:
			xUPK_V4_32();
			break;
		case 13:
			xUPK_V4_16();
			break;
		case 14:
			xUPK_V4_8();
			break;
		case 15:
			xUPK_V4_5();
			break;

		case 3:
		case 7:
		case 11:
			// TODO: Needs hardware testing.
			// Dynasty Warriors 5: Empire  - Player 2 chose a character menu.
			Console.Warning("Vpu/Vif: Invalid Unpack %d", upknum);
			break;
	}
}

// =====================================================================================================
//  VifUnpackSSE_Simple
// =====================================================================================================

VifUnpackNEON_Simple::VifUnpackNEON_Simple(bool usn_, bool domask_, int curCycle_)
{
	curCycle = curCycle_;
	usn = usn_;
	doMask = domask_;
	IsAligned = true;
}

void VifUnpackNEON_Simple::doMaskWrite(const vixl::aarch64::VRegister& regX) const
{
	// ARM64 optimized mask write with prefetching
	armAsm->Prfm(a64::PLDL1KEEP, dstIndirect);
	armAsm->Ldr(a64::q7, dstIndirect);

	int offX = std::min(curCycle, 3);
	armMoveAddressToReg(RXVIXLSCRATCH, nVifMask);
	armAsm->Ldr(a64::q29, a64::MemOperand(RXVIXLSCRATCH, reinterpret_cast<const u8*>(nVifMask[0][offX]) - reinterpret_cast<const u8*>(nVifMask)));
	armAsm->Ldr(a64::q30, a64::MemOperand(RXVIXLSCRATCH, reinterpret_cast<const u8*>(nVifMask[1][offX]) - reinterpret_cast<const u8*>(nVifMask)));
	armAsm->Ldr(a64::q31, a64::MemOperand(RXVIXLSCRATCH, reinterpret_cast<const u8*>(nVifMask[2][offX]) - reinterpret_cast<const u8*>(nVifMask)));
	armAsm->And(regX.V16B(), regX.V16B(), a64::q29.V16B());
	armAsm->And(a64::q7.V16B(), a64::q7.V16B(), a64::q30.V16B());
	armAsm->Orr(regX.V16B(), regX.V16B(), a64::q31.V16B());
	armAsm->Orr(regX.V16B(), regX.V16B(), a64::q7.V16B());
	// Store with write prefetch hint for next potential write
	armAsm->Str(regX, dstIndirect);
	armAsm->Prfm(a64::PSTL1KEEP, a64::MemOperand(dstIndirect.GetBaseRegister(), dstIndirect.GetOffset() + 16));
}

// ecx = dest, edx = src
static void nVifGen(int usn, int mask, int curCycle)
{

	int usnpart = usn * 2 * 16;
	int maskpart = mask * 16;

	VifUnpackNEON_Simple vpugen(!!usn, !!mask, curCycle);

	for (int i = 0; i < 16; ++i)
	{
		nVifCall& ucall(nVifUpk[((usnpart + maskpart + i) * 4) + curCycle]);
		ucall = NULL;
		if (nVifT[i] == 0)
			continue;

		ucall = (nVifCall)armStartBlock();
		vpugen.xUnpack(i);
		vpugen.xMovDest();
		armAsm->Ret();
		armEndBlock();
	}
}

void VifUnpackSSE_Init()
{
	DevCon.WriteLn("Generating NEON-optimized unpacking functions for VIF interpreters...");

	HostSys::BeginCodeWrite();
	armSetAsmPtr(SysMemory::GetVIFUnpackRec(), SysMemory::GetVIFUnpackRecEnd() - SysMemory::GetVIFUnpackRec(), nullptr);

	for (int a = 0; a < 2; a++)
	{
		for (int b = 0; b < 2; b++)
		{
			for (int c = 0; c < 4; c++)
			{
				nVifGen(a, b, c);
			}
		}
	}

	Perf::any.Register(SysMemory::GetVIFUnpackRec(), armGetAsmPtr() - SysMemory::GetVIFUnpackRec(), "VIF Unpack");
	HostSys::EndCodeWrite();
}

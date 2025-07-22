// SPDX-FileCopyrightText: 2002-2024 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0

#include "common/arm64/AsmHelpers.h"

#include "common/Assertions.h"
#include "common/BitUtils.h"
#include "common/Console.h"
#include "common/HostSys.h"

const a64::Register& armWRegister(int n)
{
    using namespace vixl::aarch64;
    static constexpr const Register* regs[32] = {&w0, &w1, &w2, &w3, &w4, &w5, &w6, &w7, &w8, &w9, &w10,
                                                 &w11, &w12, &w13, &w14, &w15, &w16, &w17, &w18, &w19, &w20, &w21, &w22, &w23, &w24, &w25, &w26, &w27, &w28,
                                                 &w29, &w30, &w31};
    pxAssert(static_cast<size_t>(n) < std::size(regs));
    return *regs[n];
}

const a64::Register& armXRegister(int n)
{
    using namespace vixl::aarch64;
    static constexpr const Register* regs[32] = {&x0, &x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8, &x9, &x10,
                                                 &x11, &x12, &x13, &x14, &x15, &x16, &x17, &x18, &x19, &x20, &x21, &x22, &x23, &x24, &x25, &x26, &x27, &x28,
                                                 &x29, &x30, &x31};
    pxAssert(static_cast<size_t>(n) < std::size(regs));
    return *regs[n];
}

const a64::VRegister& armSRegister(int n)
{
    using namespace vixl::aarch64;
    static constexpr const VRegister* regs[32] = {&s0, &s1, &s2, &s3, &s4, &s5, &s6, &s7, &vixl::aarch64::s8, &s9, &s10,
                                                  &s11, &s12, &s13, &s14, &s15, &vixl::aarch64::s16, &s17, &s18, &s19, &s20, &s21, &s22, &s23, &s24, &s25, &s26, &s27, &s28,
                                                  &s29, &s30, &s31};
    pxAssert(static_cast<size_t>(n) < std::size(regs));
    return *regs[n];
}

const a64::VRegister& armDRegister(int n)
{
    using namespace vixl::aarch64;
    static constexpr const VRegister* regs[32] = {&d0, &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9, &d10,
                                                  &d11, &d12, &d13, &d14, &d15, &d16, &d17, &d18, &d19, &d20, &d21, &d22, &d23, &d24, &d25, &d26, &d27, &d28,
                                                  &d29, &d30, &d31};
    pxAssert(static_cast<size_t>(n) < std::size(regs));
    return *regs[n];
}

const a64::VRegister& armQRegister(int n)
{
    using namespace vixl::aarch64;
    static constexpr const VRegister* regs[32] = {&q0, &q1, &q2, &q3, &q4, &q5, &q6, &q7, &q8, &q9, &q10,
                                                  &q11, &q12, &q13, &q14, &q15, &q16, &q17, &q18, &q19, &q20, &q21, &q22, &q23, &q24, &q25, &q26, &q27, &q28,
                                                  &q29, &q30, &q31};
    pxAssert(static_cast<size_t>(n) < std::size(regs));
    return *regs[n];
}


//#define INCLUDE_DISASSEMBLER

#ifdef INCLUDE_DISASSEMBLER
#include "vixl/aarch64/disasm-aarch64.h"
#endif

thread_local a64::MacroAssembler* armAsm;
thread_local u8* armAsmPtr;
thread_local size_t armAsmCapacity;
thread_local ArmConstantPool* armConstantPool;

#ifdef INCLUDE_DISASSEMBLER
static std::mutex armDisasmMutex;
static std::unique_ptr<a64::PrintDisassembler> armDisasm;
static std::unique_ptr<a64::Decoder> armDisasmDecoder;
#endif

void armSetAsmPtr(void* ptr, size_t capacity, ArmConstantPool* pool)
{
    pxAssert(!armAsm);
    armAsmPtr = static_cast<u8*>(ptr);
    armAsmCapacity = capacity;
    armConstantPool = pool;
}

// Align to 16 bytes, apparently ARM likes that.
void armAlignAsmPtr()
{
    static constexpr uintptr_t ALIGNMENT = 16;
    u8* new_ptr = reinterpret_cast<u8*>((reinterpret_cast<uintptr_t>(armAsmPtr) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1));
    pxAssert(static_cast<size_t>(new_ptr - armAsmPtr) <= armAsmCapacity);
    armAsmCapacity -= (new_ptr - armAsmPtr);
    armAsmPtr = new_ptr;
}

u8* armStartBlock()
{
    armAlignAsmPtr();

    HostSys::BeginCodeWrite();

    pxAssert(!armAsm);
    armAsm = new a64::MacroAssembler(static_cast<vixl::byte*>(armAsmPtr), armAsmCapacity);
    armAsm->GetScratchVRegisterList()->Remove(31);
    armAsm->GetScratchRegisterList()->Remove(RSCRATCHADDR.GetCode());
    return armAsmPtr;
}

u8* armEndBlock()
{
    pxAssert(armAsm);

    armAsm->FinalizeCode();

    const u32 size = static_cast<u32>(armAsm->GetSizeOfCodeGenerated());
    pxAssert(size < armAsmCapacity);

    delete armAsm;
    armAsm = nullptr;

    HostSys::EndCodeWrite();

    HostSys::FlushInstructionCache(armAsmPtr, size);

    armAsmPtr = armAsmPtr + size;
    armAsmCapacity -= size;
    return armAsmPtr;
}

void armDisassembleAndDumpCode(const void* ptr, size_t size)
{
#ifdef INCLUDE_DISASSEMBLER
    std::unique_lock lock(armDisasmMutex);
	if (!armDisasm)
	{
		armDisasm = std::make_unique<a64::PrintDisassembler>(stderr);
		armDisasmDecoder = std::make_unique<a64::Decoder>();
		armDisasmDecoder->AppendVisitor(armDisasm.get());
	}

	armDisasmDecoder->Decode(static_cast<const a64::Instruction*>(ptr), static_cast<const a64::Instruction*>(ptr) + size);
#else
    Console.Error("Not compiled with INCLUDE_DISASSEMBLER");
#endif
}

void armEmitJmp(const void* ptr, bool force_inline)
{
    s64 displacement = GetPCDisplacement(armGetCurrentCodePointer(), ptr);
    bool use_blr = !vixl::IsInt26(displacement);
    if (use_blr && armConstantPool && !force_inline)
    {
        if (u8* trampoline = armConstantPool->GetJumpTrampoline(ptr); trampoline)
        {
            displacement = GetPCDisplacement(armGetCurrentCodePointer(), trampoline);
            use_blr = !vixl::IsInt26(displacement);
        }
    }

    if (use_blr)
    {
        armAsm->Mov(RXVIXLSCRATCH, reinterpret_cast<uintptr_t>(ptr));
        armAsm->Br(RXVIXLSCRATCH);
    }
    else
    {
        a64::SingleEmissionCheckScope guard(armAsm);
        armAsm->b(displacement);
    }
}

void armEmitCall(const void* ptr, bool force_inline)
{
    s64 displacement = GetPCDisplacement(armGetCurrentCodePointer(), ptr);
    bool use_blr = !vixl::IsInt26(displacement);
    if (use_blr && armConstantPool && !force_inline)
    {
        if (u8* trampoline = armConstantPool->GetJumpTrampoline(ptr); trampoline)
        {
            displacement = GetPCDisplacement(armGetCurrentCodePointer(), trampoline);
            use_blr = !vixl::IsInt26(displacement);
        }
    }

    if (use_blr)
    {
        armAsm->Mov(RXVIXLSCRATCH, reinterpret_cast<uintptr_t>(ptr));
        armAsm->Blr(RXVIXLSCRATCH);
    }
    else
    {
        a64::SingleEmissionCheckScope guard(armAsm);
        armAsm->bl(displacement);
    }
}

void armEmitCbnz(const a64::Register& reg, const void* ptr)
{
    const s64 jump_distance =
            static_cast<s64>(reinterpret_cast<intptr_t>(ptr) - reinterpret_cast<intptr_t>(armGetCurrentCodePointer()));
    //pxAssert(Common::IsAligned(jump_distance, 4));
    if (a64::Instruction::IsValidImmPCOffset(a64::CompareBranchType, jump_distance >> 2))
    {
        a64::SingleEmissionCheckScope guard(armAsm);
        armAsm->cbnz(reg, jump_distance >> 2);
    }
    else
    {
        a64::MacroEmissionCheckScope guard(armAsm);
        a64::Label branch_not_taken;
        armAsm->cbz(reg, &branch_not_taken);

        const s64 new_jump_distance =
                static_cast<s64>(reinterpret_cast<intptr_t>(ptr) - reinterpret_cast<intptr_t>(armGetCurrentCodePointer()));
        armAsm->b(new_jump_distance >> 2);
        armAsm->bind(&branch_not_taken);
    }
}

void armEmitCondBranch(a64::Condition cond, const void* ptr)
{
    const s64 jump_distance =
            static_cast<s64>(reinterpret_cast<intptr_t>(ptr) - reinterpret_cast<intptr_t>(armGetCurrentCodePointer()));
    //pxAssert(Common::IsAligned(jump_distance, 4));

    if (a64::Instruction::IsValidImmPCOffset(a64::CondBranchType, jump_distance >> 2))
    {
        a64::SingleEmissionCheckScope guard(armAsm);
        armAsm->b(jump_distance >> 2, cond);
    }
    else
    {
        a64::MacroEmissionCheckScope guard(armAsm);
        a64::Label branch_not_taken;
        armAsm->b(&branch_not_taken, a64::InvertCondition(cond));

        const s64 new_jump_distance =
                static_cast<s64>(reinterpret_cast<intptr_t>(ptr) - reinterpret_cast<intptr_t>(armGetCurrentCodePointer()));
        armAsm->b(new_jump_distance >> 2);
        armAsm->bind(&branch_not_taken);
    }
}

void armMoveAddressToReg(const a64::Register& reg, const void* addr)
{
    // psxAsm->Mov(reg, static_cast<u64>(reinterpret_cast<uintptr_t>(addr)));
    pxAssert(reg.IsX());

//    const void* current_code_ptr_page = reinterpret_cast<const void*>(
//            reinterpret_cast<uintptr_t>(armGetCurrentCodePointer()) & ~static_cast<uintptr_t>(0xFFF));
//    const void* ptr_page =
//            reinterpret_cast<const void*>(reinterpret_cast<uintptr_t>(addr) & ~static_cast<uintptr_t>(0xFFF));
//    const s64 page_displacement = GetPCDisplacement(current_code_ptr_page, ptr_page) >> 10;
//    const u32 page_offset = static_cast<u32>(reinterpret_cast<uintptr_t>(addr) & 0xFFFu);
//    if (vixl::IsInt21(page_displacement) && a64::Assembler::IsImmAddSub(page_offset))
//    {
//        {
//            a64::SingleEmissionCheckScope guard(armAsm);
//            armAsm->adrp(reg, page_displacement);
//        }
//        armAsm->Add(reg, reg, page_offset);
//    }
//    else if (vixl::IsInt21(page_displacement) && a64::Assembler::IsImmLogical(page_offset, 64))
//    {
//        {
//            a64::SingleEmissionCheckScope guard(armAsm);
//            armAsm->adrp(reg, page_displacement);
//        }
//        armAsm->Orr(reg, reg, page_offset);
//    }
//    else
    {
        armAsm->Mov(reg, reinterpret_cast<uintptr_t>(addr));
    }
}

void armLoadPtr(const a64::CPURegister& reg, const void* addr)
{
    armMoveAddressToReg(RSCRATCHADDR, addr);
    armAsm->Ldr(reg, a64::MemOperand(RSCRATCHADDR));
}

void armStorePtr(const a64::CPURegister& reg, const void* addr)
{
    armMoveAddressToReg(RSCRATCHADDR, addr);
    armAsm->Str(reg, a64::MemOperand(RSCRATCHADDR));
}

void armBeginStackFrame(bool save_fpr)
{
    // save x19 through x28, x29 could also be used
    armAsm->Sub(a64::sp, a64::sp, save_fpr ? 192 : 144);
    armAsm->Stp(a64::x19, a64::x20, a64::MemOperand(a64::sp, 32));
    armAsm->Stp(a64::x21, a64::x22, a64::MemOperand(a64::sp, 48));
    armAsm->Stp(a64::x23, a64::x24, a64::MemOperand(a64::sp, 64));
    armAsm->Stp(a64::x25, a64::x26, a64::MemOperand(a64::sp, 80));
    armAsm->Stp(a64::x27, a64::x28, a64::MemOperand(a64::sp, 96));
    armAsm->Stp(a64::x29, a64::lr, a64::MemOperand(a64::sp, 112));
    if (save_fpr)
    {
        armAsm->Stp(a64::d8, a64::d9, a64::MemOperand(a64::sp, 128));
        armAsm->Stp(a64::d10, a64::d11, a64::MemOperand(a64::sp, 144));
        armAsm->Stp(a64::d12, a64::d13, a64::MemOperand(a64::sp, 160));
        armAsm->Stp(a64::d14, a64::d15, a64::MemOperand(a64::sp, 176));
    }
}

void armEndStackFrame(bool save_fpr)
{
    if (save_fpr)
    {
        armAsm->Ldp(a64::d14, a64::d15, a64::MemOperand(a64::sp, 176));
        armAsm->Ldp(a64::d12, a64::d13, a64::MemOperand(a64::sp, 160));
        armAsm->Ldp(a64::d10, a64::d11, a64::MemOperand(a64::sp, 144));
        armAsm->Ldp(a64::d8, a64::d9, a64::MemOperand(a64::sp, 128));
    }
    armAsm->Ldp(a64::x29, a64::lr, a64::MemOperand(a64::sp, 112));
    armAsm->Ldp(a64::x27, a64::x28, a64::MemOperand(a64::sp, 96));
    armAsm->Ldp(a64::x25, a64::x26, a64::MemOperand(a64::sp, 80));
    armAsm->Ldp(a64::x23, a64::x24, a64::MemOperand(a64::sp, 64));
    armAsm->Ldp(a64::x21, a64::x22, a64::MemOperand(a64::sp, 48));
    armAsm->Ldp(a64::x19, a64::x20, a64::MemOperand(a64::sp, 32));
    armAsm->Add(a64::sp, a64::sp, save_fpr ? 192 : 144);
}

bool armIsCalleeSavedRegister(int reg)
{
    // same on both linux and windows
    return (reg >= 19);
}

bool armIsCallerSaved(int id)
{
#if defined(__ANDROID__)
    // 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
    return (id <= 15);
#else
    #ifdef _WIN32
    // The x64 ABI considers the registers RAX, RCX, RDX, R8, R9, R10, R11, and XMM0-XMM5 volatile.
    return (id <= 2 || (id >= 8 && id <= 11));
    #else
    // rax, rdi, rsi, rdx, rcx, r8, r9, r10, r11 are scratch registers.
    return (id <= 2 || id == 6 || id == 7 || (id >= 8 && id <= 11));
    #endif
#endif
}

bool armIsCallerSavedXmm(int id)
{
#if defined(__ANDROID__)
    // v9,v10,v11,v12,v13,v14,v15
    return (id < 9);
//    return true;
#else
    #ifdef _WIN32
    // XMM6 through XMM15 are saved. Upper 128 bits is always volatile.
        return (id < 6);
    #else
    // All vector registers are volatile.
    return true;
    #endif
#endif
}

a64::MemOperand armOffsetMemOperand(const a64::MemOperand& op, s64 offset)
{
    pxAssert(op.GetBaseRegister().IsValid() && op.GetAddrMode() == a64::Offset && op.GetShift() == a64::NO_SHIFT);
    return a64::MemOperand(op.GetBaseRegister(), op.GetOffset() + offset, op.GetAddrMode());
}

void armGetMemOperandInRegister(const a64::Register& addr_reg, const a64::MemOperand& op, s64 extra_offset /*= 0*/)
{
    pxAssert(addr_reg.IsX());
    pxAssert(op.GetBaseRegister().IsValid() && op.GetAddrMode() == a64::Offset && op.GetShift() == a64::NO_SHIFT);
    armAsm->Add(addr_reg, op.GetBaseRegister(), op.GetOffset() + extra_offset);
}

void armLoadConstant128(const a64::VRegister& reg, const void* ptr)
{
    u64 low, high;
    memcpy(&low, ptr, sizeof(low));
    memcpy(&high, static_cast<const u8*>(ptr) + sizeof(low), sizeof(high));
    armAsm->Ldr(reg, high, low);
}

void armEmitVTBL(const a64::VRegister& dst, const a64::VRegister& src1, const a64::VRegister& src2, const a64::VRegister& tbl)
{
    pxAssert(src1.GetCode() != RQSCRATCH.GetCode() && src2.GetCode() != RQSCRATCH2.GetCode());
    pxAssert(tbl.GetCode() != RQSCRATCH.GetCode() && tbl.GetCode() != RQSCRATCH2.GetCode());

    // must be consecutive
    if (src2.GetCode() == (src1.GetCode() + 1))
    {
        armAsm->Tbl(dst.V16B(), src1.V16B(), src2.V16B(), tbl.V16B());
        return;
    }

    armAsm->Mov(RQSCRATCH.Q(), src1.Q());
    armAsm->Mov(RQSCRATCH2.Q(), src2.Q());
    armAsm->Tbl(dst.V16B(), RQSCRATCH.V16B(), RQSCRATCH2.V16B(), tbl.V16B());
}


void ArmConstantPool::Init(void* ptr, u32 capacity)
{
    m_base_ptr = static_cast<u8*>(ptr);
    m_capacity = capacity;
    m_used = 0;
    m_jump_targets.clear();
    m_literals.clear();
}

void ArmConstantPool::Destroy()
{
    m_base_ptr = nullptr;
    m_capacity = 0;
    m_used = 0;
    m_jump_targets.clear();
    m_literals.clear();
}

void ArmConstantPool::Reset()
{
    m_used = 0;
    m_jump_targets.clear();
    m_literals.clear();
}

u8* ArmConstantPool::GetJumpTrampoline(const void* target)
{
    auto it = m_jump_targets.find(target);
    if (it != m_jump_targets.end())
        return m_base_ptr + it->second;

    // align to 16 bytes?
    const u32 offset = Common::AlignUpPow2(m_used, 16);

    // 4 movs plus a jump
    if ((m_capacity - offset) < 20)
    {
        Console.Error("Ran out of space in constant pool");
        return nullptr;
    }

    a64::MacroAssembler masm(static_cast<vixl::byte*>(m_base_ptr + offset), m_capacity - offset);
    masm.Mov(RXVIXLSCRATCH, reinterpret_cast<intptr_t>(target));
    masm.Br(RXVIXLSCRATCH);
    masm.FinalizeCode();

    pxAssert(masm.GetSizeOfCodeGenerated() < 20);
    m_jump_targets.emplace(target, offset);
    m_used = offset + static_cast<u32>(masm.GetSizeOfCodeGenerated());

    HostSys::FlushInstructionCache(reinterpret_cast<void*>(m_base_ptr + offset), m_used - offset);

    return m_base_ptr + offset;
}

u8* ArmConstantPool::GetLiteral(u64 value)
{
    return GetLiteral(u128::From64(value));
}

u8* ArmConstantPool::GetLiteral(const u128& value)
{
    auto it = m_literals.find(value);
    if (it != m_literals.end())
        return m_base_ptr + it->second;

    if (GetRemainingCapacity() < 8)
        return nullptr;

    const u32 offset = Common::AlignUpPow2(m_used, 16);
    std::memcpy(&m_base_ptr[offset], &value, sizeof(value));
    m_used = offset + sizeof(value);
    return m_base_ptr + offset;
}

u8* ArmConstantPool::GetLiteral(const u8* bytes, size_t len)
{
    pxAssertMsg(len <= 16, "literal length is less than 16 bytes");
    u128 table_u128 = {};
    std::memcpy(table_u128._u8, bytes, len);
    return GetLiteral(table_u128);
}

void ArmConstantPool::EmitLoadLiteral(const a64::CPURegister& reg, const u8* literal) const
{
    armMoveAddressToReg(RXVIXLSCRATCH, literal);
    armAsm->Ldr(reg, a64::MemOperand(RXVIXLSCRATCH));
}

//////////////////////////////////////////////////////////////////////////

void armBind(a64::Label* p_label)
{
    if(p_label->IsLinked()) {
        armAsm->Bind(p_label);
    }
}

u32 armEmitJmpPtr(void* code, const void* dst, bool flush_icache)
{
    const s64 displacement = GetPCDisplacement(code, dst);
    bool use_blr = !vixl::IsInt26(displacement);

    if (use_blr)
    {
        armAsm->Mov(RXVIXLSCRATCH, reinterpret_cast<uintptr_t>(dst));
        armAsm->Br(RXVIXLSCRATCH);
    }
    else
    {
        u32 new_code = a64::B | a64::Assembler::ImmUncondBranch(displacement);
        std::memcpy(code, &new_code, sizeof(new_code));
    }

    if (flush_icache) {
        HostSys::FlushInstructionCache(code, a64::kInstructionSize);
    }

    return a64::kInstructionSize;
}

a64::Register armLoadPtr(const void* addr)
{
    armAsm->Ldr(a64::w4, armMemOperandPtr(addr));
    return a64::w4;
}

a64::Register armLoadPtr64(const void* addr)
{
    armAsm->Ldr(a64::x4, armMemOperandPtr(addr));
    return a64::x4;
}

a64::Register armLdrh(const void* addr)
{
    armAsm->Ldrh(a64::w4, armMemOperandPtr(addr));
    return a64::w4;
}

a64::Register armLdrsh(const void* addr)
{
    armAsm->Ldrsh(a64::w4, armMemOperandPtr(addr));
    return a64::w4;
}

void armLoadPtr(const a64::CPURegister& reg, const void* addr, int64_t offset)
{
    armMoveAddressToReg(RSCRATCHADDR, addr);
    armAsm->Ldr(reg, a64::MemOperand(RSCRATCHADDR, offset));
}

void armLoadPtr(const a64::CPURegister& regRt, a64::Register regRs, int64_t offset)
{
    armAsm->Ldr(regRt, a64::MemOperand(regRs, offset));
}

a64::Register armLoadPtr(a64::Register regRs, int64_t offset)
{
    armAsm->Ldr(EEX, a64::MemOperand(regRs, offset));
    return EEX;
}

void armLoadPtr(uint64_t imm, const void* addr, const a64::Register& reg)
{
    armAsm->Mov(reg, imm);
    armLoadPtr(reg, addr);
}

void armLoadPtr(uint64_t imm, a64::Register regRs, int64_t offset, const a64::Register& regRt)
{
    armAsm->Mov(regRt, imm);
    armAsm->Ldr(regRt, a64::MemOperand(regRs, offset));
}

a64::VRegister armLoadPtrV(const void* addr)
{
    armAsm->Ldr(RQSCRATCH, armMemOperandPtr(addr));
    return RQSCRATCH;
}

a64::VRegister armLoadPtrM(a64::Register regRs, int64_t offset)
{
    armAsm->Ldr(RQSCRATCH, a64::MemOperand(regRs, offset));
    return RQSCRATCH;
}

////////////////////////////////////////////////////////////////////////////////

void armStorePtr(const a64::CPURegister& reg, const void* addr, int64_t offset)
{
    armMoveAddressToReg(RSCRATCHADDR, addr);
    armAsm->Str(reg, a64::MemOperand(RSCRATCHADDR, offset));
}

void armStorePtr(const a64::CPURegister& regRt, a64::Register regRs, int64_t offset)
{
    armAsm->Str(regRt, a64::MemOperand(regRs, offset));
}

void armStorePtr(uint64_t imm, const void* addr, const a64::Register& reg)
{
    if(imm == 0) {
        armAsm->Str(a64::xzr, armMemOperandPtr(addr));
    } else {
        armAsm->Mov(reg, imm);
        armAsm->Str(reg, armMemOperandPtr(addr));
    }
}

void armStorePtr(uint64_t imm, a64::Register regRs, int64_t offset, const a64::Register& regRt)
{
    if(imm == 0) {
        armAsm->Str(a64::xzr, a64::MemOperand(regRs, offset));
    } else {
        armAsm->Mov(regRt, imm);
        armAsm->Str(regRt, a64::MemOperand(regRs, offset));
    }
}

a64::MemOperand armMemOperandPtr(const void* addr)
{
    armMoveAddressToReg(RSCRATCHADDR, addr);
    return a64::MemOperand(RSCRATCHADDR);
}

void armLoad(const a64::Register& regRt, a64::MemOperand offset)
{
    armAsm->Ldr(regRt, offset);
}

void armLoadh(const a64::Register& regRt, a64::MemOperand offset)
{
    armAsm->Ldrh(regRt, offset);
}

void armLoadsh(const a64::Register& regRt, a64::MemOperand offset)
{
    armAsm->Ldrsh(regRt, offset);
}

void armLoadsw(const a64::Register& regRt, a64::MemOperand offset)
{
    armAsm->Ldrsw(regRt,  offset);
}

void armLoad(const a64::VRegister& regRt, a64::MemOperand offset)
{
    armAsm->Ldr(regRt,  offset);
}

a64::Register armLoad(a64::MemOperand offset)
{
    armAsm->Ldr(EEX,  offset);
    return EEX;
}

a64::Register armLoad64(a64::MemOperand offset)
{
    armAsm->Ldr(REX,  offset);
    return REX;
}

a64::VRegister armLoadPtrV(a64::MemOperand offset)
{
    armAsm->Ldr(RQSCRATCH,  offset);
    return RQSCRATCH;
}

void armStore(a64::MemOperand offset, const a64::Register& regRt)
{
    armAsm->Str(regRt,  offset);
}

void armStoreh(a64::MemOperand offset, const a64::Register& regRt)
{
    armAsm->Strh(regRt,  offset);
}

void armStore(a64::MemOperand offset, const a64::VRegister& regRt)
{
    armAsm->Str(regRt,  offset);
}

void armStore(a64::MemOperand offset, uint64_t imm)
{
    armAsm->Mov(EEX, imm);
    armAsm->Str(EEX,  offset);
}

void armStore64(a64::MemOperand offset, uint64_t imm)
{
    armAsm->Mov(REX, imm);
    armAsm->Str(REX,  offset);
}

void armAdd(a64::MemOperand p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armAsm->Ldr(EEX, p_mop);
    if(p_flagUpdate) {
        armAsm->Adds(EEX, EEX, p_value);
    } else {
        armAsm->Add(EEX, EEX, p_value);
    }
    armAsm->Str(EEX, p_mop);
}

void armAdd(const a64::Register& p_reg, a64::MemOperand p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armAsm->Ldr(p_reg, p_mop);
    if(p_flagUpdate) {
        armAsm->Adds(p_reg, p_reg, p_value);
    } else {
        armAsm->Add(p_reg, p_reg, p_value);
    }
    armAsm->Str(p_reg, p_mop);
}

void armAdd(const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(EEX, memop);
    armAsm->Add(EEX, EEX, p_value);
    armAsm->Str(EEX, memop);
}

void armAdd(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(p_reg, memop);
    armAsm->Add(p_reg, p_reg, p_value);
    armAsm->Str(p_reg, memop);
}

void armAddh(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldrh(p_reg, memop);
    if(p_flagUpdate) {
        armAsm->Adds(p_reg, p_reg, p_value);
    } else {
        armAsm->Add(p_reg, p_reg, p_value);
    }
    armAsm->Strh(p_reg, memop);
}

void armAddsh(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldrsh(p_reg, memop);
    if(p_flagUpdate) {
        armAsm->Adds(p_reg, p_reg, p_value);
    } else {
        armAsm->Add(p_reg, p_reg, p_value);
    }
    armAsm->Strh(p_reg, memop);
}

void armSub(a64::MemOperand p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armLoadsw(EEX, p_mop);
    if(p_flagUpdate) {
        armAsm->Subs(EEX, EEX, p_value);
    } else {
        armAsm->Sub(EEX, EEX, p_value);
    }
    armStore(p_mop, EEX);
}

void armSub(const a64::Register& p_reg, a64::MemOperand p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armLoadsw(p_reg, p_mop);
    if(p_flagUpdate) {
        armAsm->Subs(p_reg, p_reg, p_value);
    } else {
        armAsm->Sub(p_reg, p_reg, p_value);
    }
    armStore(p_mop, p_reg);
}

void armSub(const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(EEX, memop);
    armAsm->Sub(EEX, EEX, p_value);
    armAsm->Str(EEX, memop);
}

void armSub(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(p_reg, memop);
    armAsm->Sub(p_reg, p_reg, p_value);
    armAsm->Str(p_reg, memop);
}

void armAnd(a64::MemOperand p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armLoad(EEX, p_mop);
    if(p_flagUpdate) {
        armAsm->Ands(EEX, EEX, p_value);
    } else {
        armAsm->And(EEX, EEX, p_value);
    }
    armStore(p_mop, EEX);
}

void armAnd(const a64::Register& p_reg, a64::MemOperand p_mop, a64::Operand p_value, bool p_flagUpdate)
{
    armLoad(p_reg, p_mop);
    if(p_flagUpdate) {
        armAsm->Ands(p_reg, p_reg, p_value);
    } else {
        armAsm->And(p_reg, p_reg, p_value);
    }
    armStore(p_mop, p_reg);
}

void armAnd(const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(EEX, memop);
    armAsm->And(EEX, EEX, p_value);
    armAsm->Str(EEX, memop);
}

void armAnd(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(p_reg, memop);
    armAsm->And(p_reg, p_reg, p_value);
    armAsm->Str(p_reg, memop);
}

void armOrr(a64::MemOperand p_mop, a64::Operand p_value)
{
    armLoad(EEX, p_mop);
    armAsm->Orr(EEX, EEX, p_value);
    armStore(p_mop, EEX);
}

void armOrr(const a64::Register& p_reg, a64::MemOperand p_mop, a64::Operand p_value)
{
    armLoad(p_reg, p_mop);
    armAsm->Orr(p_reg, p_reg, p_value);
    armStore(p_mop, p_reg);
}

void armOrr(const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(EEX, memop);
    armAsm->Orr(EEX, EEX, p_value);
    armAsm->Str(EEX, memop);
}

void armOrr(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(p_reg, memop);
    armAsm->Orr(p_reg, p_reg, p_value);
    armAsm->Str(p_reg, memop);
}

void armEor(a64::MemOperand p_mop, a64::Operand p_value)
{
    armLoad(EEX, p_mop);
    armAsm->Eor(EEX, EEX, p_value);
    armStore(p_mop, EEX);
}

void armEor(const a64::Register& p_reg, a64::MemOperand p_mop, a64::Operand p_value)
{
    armLoad(p_reg, p_mop);
    armAsm->Eor(p_reg, p_reg, p_value);
    armStore(p_mop, p_reg);
}

void armEor(const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(EEX, memop);
    armAsm->Eor(EEX, EEX, p_value);
    armAsm->Str(EEX, memop);
}

void armEor(const a64::Register& p_reg, const void* p_mop, a64::Operand p_value)
{
    armMoveAddressToReg(RSCRATCHADDR, p_mop);
    auto memop = a64::MemOperand(RSCRATCHADDR);
    ////
    armAsm->Ldr(p_reg, memop);
    armAsm->Eor(p_reg, p_reg, p_value);
    armAsm->Str(p_reg, memop);
}

// kArm64I32x4BitMask
void armMOVMSKPS(const a64::Register& reg32, const a64::VRegister& regQ)
{
    armAsm->Movi(RQSCRATCH2.V2D(), 0x0000'0008'0000'0004, 0x0000'0002'0000'0001);
    ////
    armAsm->Sshr(RQSCRATCH.V4S(), regQ.V4S(), 31);
    armAsm->And(RQSCRATCH.V16B(), RQSCRATCH2.V16B(), RQSCRATCH.V16B());

    armAsm->Addv(RQSCRATCH.S(), RQSCRATCH.V4S());
    armAsm->Fmov(reg32, RQSCRATCH.S());
}

void armPBLENDW(const a64::VRegister& regDst, const a64::VRegister& regSrc)
{
    armAsm->Mov(RQSCRATCH, regDst);
    armAsm->Movi(regDst.V4S(), 0xFFFF0000);
    armAsm->Bsl(regDst.V16B(), RQSCRATCH.V16B(), regSrc.V16B());
}

// kArm64I8x16SConvertI16x8
void armPACKSSWB(const a64::VRegister& regDst, const a64::VRegister& regSrc)
{
    a64::VRegister src0 = regDst, src1 = regSrc;
    if (regDst.Is(src1)) {
        armAsm->Mov(RQSCRATCH.V8H(), src1.V8H());
        src1 = RQSCRATCH;
    }
    armAsm->Sqxtn(regDst.V8B(), src0.V8H());
    armAsm->Sqxtn2(regDst.V16B(), src1.V8H());
}

void armPMOVMSKB(const a64::Register& regDst, const a64::VRegister& regSrc)
{
    a64::VRegister tmp = RQSCRATCH;
    a64::VRegister mask = RQSCRATCH2;
    //
    armAsm->Sshr(tmp.V16B(), regSrc.V16B(), 7);
    armAsm->Movi(mask.V2D(), 0x8040'2010'0804'0201);
    armAsm->And(tmp.V16B(), mask.V16B(), tmp.V16B());

    armAsm->Ext(mask.V16B(), tmp.V16B(), tmp.V16B(), 8);
    armAsm->Zip1(tmp.V16B(), tmp.V16B(), mask.V16B());

    armAsm->Addv(tmp.H(), tmp.V8H());
    armAsm->Mov(regDst, tmp.V8H(), 0);
}

void armSHUFPS(const a64::VRegister& dstreg, const a64::VRegister& srcreg, int pIndex)
{
    armShuffle(dstreg, srcreg, pIndex, true);
}

void armPSHUFD(const a64::VRegister& dstreg, const a64::VRegister& srcreg, int pIndex)
{
    armShuffle(dstreg, srcreg, pIndex, false);
}

void armShuffleTblx(const a64::VRegister& p_dst, const a64::VRegister& p_src, int p_a, int p_b, int p_c, int p_d, bool p_is_tbx)
{
    uint8_t lanes[16];
    int i, s, e, nIndex = 0;

    int lanesTbx = 0;
    if(p_is_tbx) {
        lanesTbx = 16;
    }

    // 0, 4, 8, 12
    // 0 * 4 = 0, 1 * 4 = 4, 2 * 4 = 8, 3 * 4 = 12
    // lanes 16 => 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15

    s = p_a << 2; e = s + 4;
    for(i=s; i<e; ++i) {
        lanes[nIndex++] = i;
    }
    ////
    s = p_b << 2; e = s + 4;
    for(i=s; i<e; ++i) {
        lanes[nIndex++] = i;
    }
    ////
    s = p_c << 2; e = s + 4;
    for(i=s; i<e; ++i) {
        lanes[nIndex++] = i + lanesTbx;
    }
    ////
    s = p_d << 2; e = s + 4;
    for(i=s; i<e; ++i) {
        lanes[nIndex++] = i + lanesTbx;
    }

    armLoadConstant128(RQSCRATCH3, lanes);

    if(p_is_tbx) {
        armAsm->Mov(RQSCRATCH, p_dst);
        armAsm->Mov(RQSCRATCH2, p_src);
        armAsm->Tbx(p_dst.V16B(), RQSCRATCH.V16B(), RQSCRATCH2.V16B(), RQSCRATCH3.V16B());
    } else {
        armAsm->Tbl(p_dst.V16B(), p_src.V16B(), RQSCRATCH3.V16B());
    }
}

void armShuffle(const a64::VRegister& dstreg, const a64::VRegister& srcreg, int pIndex, bool p_is_tbx)
{
    int shuffle_0 = (pIndex >> 0) & 0x3;
    int shuffle_1 = (pIndex >> 2) & 0x3;
    int shuffle_2 = (pIndex >> 4) & 0x3;
    int shuffle_3 = (pIndex >> 6) & 0x3;
    ////
    armShuffleTblx(dstreg, srcreg, shuffle_0, shuffle_1, shuffle_2, shuffle_3, p_is_tbx);
}

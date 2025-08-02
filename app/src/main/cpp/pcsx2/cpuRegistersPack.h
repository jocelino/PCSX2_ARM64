//
// Created by k2154 on 2025-07-30.
//

#ifndef PCSX2_CPUREGISTERSPACK_H
#define PCSX2_CPUREGISTERSPACK_H

#include "R5900Def.h"
#include "R3000ADef.h"
#include "VUDef.h"

struct cpuRegistersPack
{
    alignas(16) cpuRegisters cpuRegs{};
    alignas(16) fpuRegisters fpuRegs{};
    alignas(16) psxRegisters psxRegs{};
    alignas(16) VURegs vuRegs[2];
};
alignas(16) extern cpuRegistersPack g_cpuRegistersPack;
////
static cpuRegisters& cpuRegs = g_cpuRegistersPack.cpuRegs;
static fpuRegisters& fpuRegs = g_cpuRegistersPack.fpuRegs;
static psxRegisters& psxRegs = g_cpuRegistersPack.psxRegs;
static VURegs& VU0 = g_cpuRegistersPack.vuRegs[0];
static VURegs& VU1 = g_cpuRegistersPack.vuRegs[1];

#endif //PCSX2_CPUREGISTERSPACK_H

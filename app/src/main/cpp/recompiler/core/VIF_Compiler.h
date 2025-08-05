// ARM64 VIF Compiler Header
// Compiles VIF transfer operations to optimized ARM64 code

#pragma once

#include "VIF_UnpackNEON.h"

namespace ARM64Recompiler {

struct VIFTransferInfo {
    void* dst;
    const void* src;
    size_t count;
    VIFUnpackMode mode;
    bool usn;
    bool canUseBatch;
};

// VIF compilation functions
void* compileVIFTransfer(const VIFTransferInfo* info);
bool optimizeVIFTransfer(VIFTransferInfo* info);

} // namespace ARM64Recompiler
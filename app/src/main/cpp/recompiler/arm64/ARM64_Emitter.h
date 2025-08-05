// ARM64 Code Emitter Header
// Generates optimized ARM64 assembly code

#pragma once

#include <cstdint>
#include <cstddef>

namespace ARM64Recompiler {

class ARM64Emitter;

// Emitter creation and destruction
ARM64Emitter* createEmitter(uint8_t* buffer, size_t size);
void destroyEmitter(ARM64Emitter* emitter);

} // namespace ARM64Recompiler
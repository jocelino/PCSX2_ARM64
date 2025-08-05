// ARM64 Code Generation Implementation
// High-level code generation utilities

#include "ARM64_CodeGen.h"
#include "ARM64_Emitter.h"

namespace ARM64Recompiler {

void generatePrologue(uint8_t* code, size_t& offset) {
    ARM64Emitter* emitter = createEmitter(code + offset, 1024);
    
    // Save frame pointer and link register
    // stp x29, x30, [sp, #-16]!
    emitter->emitInstruction(0xA9BF7BFD);
    
    // mov x29, sp
    emitter->emitInstruction(0x910003FD);
    
    offset += emitter->getCurrentOffset();
    destroyEmitter(emitter);
}

void generateEpilogue(uint8_t* code, size_t& offset) {
    ARM64Emitter* emitter = createEmitter(code + offset, 1024);
    
    // ldp x29, x30, [sp], #16
    emitter->emitInstruction(0xA8C17BFD);
    
    // ret
    emitter->emitRet();
    
    offset += emitter->getCurrentOffset();
    destroyEmitter(emitter);
}

void generateVectorOperation(uint8_t* code, size_t& offset, VectorOp op, 
                           uint32_t dst, uint32_t src1, uint32_t src2) {
    ARM64Emitter* emitter = createEmitter(code + offset, 1024);
    
    switch (op) {
        case VECTOR_ADD:
            emitter->emitFAdd(dst, src1, src2);
            break;
        case VECTOR_SUB:
            emitter->emitFSub(dst, src1, src2);
            break;
        case VECTOR_MUL:
            emitter->emitFMul(dst, src1, src2);
            break;
        default:
            emitter->emitNop();
            break;
    }
    
    offset += emitter->getCurrentOffset();
    destroyEmitter(emitter);
}

} // namespace ARM64Recompiler
// ARM64 Assembly JIT Helper Functions
// Optimized assembly routines for PCSX2 recompiler

.text
.align 4

// Fast function prologue/epilogue for generated code
.global arm64_jit_prologue
.global arm64_jit_epilogue
.global arm64_context_switch
.global arm64_cache_flush

// Function prologue - save callee-saved registers
arm64_jit_prologue:
    // Save frame pointer and link register
    stp     x29, x30, [sp, #-16]!
    mov     x29, sp
    
    // Save callee-saved registers (x19-x28)
    stp     x19, x20, [sp, #-16]!
    stp     x21, x22, [sp, #-16]!
    stp     x23, x24, [sp, #-16]!
    stp     x25, x26, [sp, #-16]!
    stp     x27, x28, [sp, #-16]!
    
    // Save NEON callee-saved registers (d8-d15)
    stp     d8,  d9,  [sp, #-16]!
    stp     d10, d11, [sp, #-16]!
    stp     d12, d13, [sp, #-16]!
    stp     d14, d15, [sp, #-16]!
    
    ret

// Function epilogue - restore callee-saved registers
arm64_jit_epilogue:
    // Restore NEON callee-saved registers
    ldp     d14, d15, [sp], #16
    ldp     d12, d13, [sp], #16
    ldp     d10, d11, [sp], #16
    ldp     d8,  d9,  [sp], #16
    
    // Restore callee-saved registers
    ldp     x27, x28, [sp], #16
    ldp     x25, x26, [sp], #16
    ldp     x23, x24, [sp], #16
    ldp     x21, x22, [sp], #16
    ldp     x19, x20, [sp], #16
    
    // Restore frame pointer and link register
    ldp     x29, x30, [sp], #16
    ret

// Fast context switch between emulated and host code
arm64_context_switch:
    // Save current context
    stp     x0, x1, [sp, #-16]!
    
    // Load new context from x0 (context pointer)
    ldp     x2, x3, [x0, #16]
    ldp     x4, x5, [x0, #32]
    ldp     x6, x7, [x0, #48]
    
    // Restore and jump
    ldp     x0, x1, [sp], #16
    ret

// Cache flush for generated code
// x0 = start address, x1 = size
arm64_cache_flush:
    add     x1, x0, x1          // Calculate end address
    
1:  dc      cvau, x0            // Data cache clean by VA to PoU
    add     x0, x0, #64         // Move to next cache line
    cmp     x0, x1
    b.lo    1b
    
    dsb     ish                 // Data synchronization barrier
    
    sub     x0, x0, x1          // Reset to start address
2:  ic      ivau, x0            // Instruction cache invalidate by VA to PoU
    add     x0, x0, #64         // Move to next cache line
    cmp     x0, x1
    b.lo    2b
    
    dsb     ish                 // Data synchronization barrier
    isb                         // Instruction synchronization barrier
    ret

// Fast memory copy using NEON (16-byte aligned)
.global arm64_fast_memcpy
arm64_fast_memcpy:
    // x0 = dst, x1 = src, x2 = size
    cmp     x2, #64
    b.lo    3f                  // Use regular copy for small sizes
    
    // Large copy using NEON quad-word loads/stores
1:  ldp     q0, q1, [x1], #32
    ldp     q2, q3, [x1], #32
    stp     q0, q1, [x0], #32
    stp     q2, q3, [x0], #32
    sub     x2, x2, #64
    cmp     x2, #64
    b.hs    1b
    
    // Handle remaining bytes
3:  cmp     x2, #16
    b.lo    4f
    ldr     q0, [x1], #16
    str     q0, [x0], #16
    sub     x2, x2, #16
    b       3b
    
    // Handle remaining bytes < 16
4:  cmp     x2, #8
    b.lo    5f
    ldr     x3, [x1], #8
    str     x3, [x0], #8
    sub     x2, x2, #8
    
5:  cmp     x2, #4
    b.lo    6f
    ldr     w3, [x1], #4
    str     w3, [x0], #4
    sub     x2, x2, #4
    
6:  cmp     x2, #0
    b.eq    7f
    ldrb    w3, [x1], #1
    strb    w3, [x0], #1
    sub     x2, x2, #1
    b       6b
    
7:  ret

// Fast memory set using NEON
.global arm64_fast_memset
arm64_fast_memset:
    // x0 = dst, x1 = value (byte), x2 = size
    dup     v0.16b, w1          // Duplicate byte across NEON register
    
    cmp     x2, #64
    b.lo    2f
    
    // Large memset using NEON
1:  stp     q0, q0, [x0], #32
    stp     q0, q0, [x0], #32
    sub     x2, x2, #64
    cmp     x2, #64
    b.hs    1b
    
    // Handle remaining bytes
2:  cmp     x2, #16
    b.lo    3f
    str     q0, [x0], #16
    sub     x2, x2, #16
    b       2b
    
3:  cmp     x2, #0
    b.eq    4f
    strb    w1, [x0], #1
    sub     x2, x2, #1
    b       3b
    
4:  ret

// Optimized VU register operations
.global arm64_vu_add_vectors
.global arm64_vu_mul_vectors
.global arm64_vu_dot_product

// Vector addition: dst = a + b (4x float32)
arm64_vu_add_vectors:
    // x0 = dst, x1 = a, x2 = b
    ldr     q0, [x1]            // Load vector a
    ldr     q1, [x2]            // Load vector b
    fadd    v0.4s, v0.4s, v1.4s // Add vectors
    str     q0, [x0]            // Store result
    ret

// Vector multiplication: dst = a * b (4x float32)
arm64_vu_mul_vectors:
    // x0 = dst, x1 = a, x2 = b
    ldr     q0, [x1]            // Load vector a
    ldr     q1, [x2]            // Load vector b
    fmul    v0.4s, v0.4s, v1.4s // Multiply vectors
    str     q0, [x0]            // Store result
    ret

// Dot product: return a.x*b.x + a.y*b.y + a.z*b.z (3-component)
arm64_vu_dot_product:
    // x0 = a, x1 = b, return in s0
    ldr     q0, [x0]            // Load vector a
    ldr     q1, [x1]            // Load vector b
    fmul    v0.4s, v0.4s, v1.4s // Multiply components
    
    // Sum first 3 components
    faddp   v0.4s, v0.4s, v0.4s // Pairwise add: [a+b, c+d, a+b, c+d]
    faddp   s0, v0.2s           // Add pairs: a+b+c
    ret

// High-precision reciprocal using Newton-Raphson
.global arm64_reciprocal
arm64_reciprocal:
    // s0 = input, return in s0
    frecpe  s1, s0              // Initial estimate
    frecps  s2, s1, s0          // Correction factor
    fmul    s1, s1, s2          // Refine estimate
    frecps  s2, s1, s0          // Second correction
    fmul    s0, s1, s2          // Final result
    ret

// High-precision reciprocal square root
.global arm64_rsqrt
arm64_rsqrt:
    // s0 = input, return in s0
    frsqrte s1, s0              // Initial estimate
    frsqrts s2, s1, s0          // Correction factor
    fmul    s1, s1, s2          // Refine estimate
    frsqrts s2, s1, s0          // Second correction
    fmul    s0, s1, s2          // Final result
    ret

.end
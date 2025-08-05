// ARM64 Recompiler Integration Test
// Test and demonstration of ARM64 optimized functions

#include "Common.h"

#ifdef ARM64_RECOMPILER_ENABLED
#include "recompiler/ARM64Recompiler.h"
#include "common/Console.h"

namespace ARM64RecompilerTest {

void testMemoryOperations() {
    Console.WriteLn("ARM64 Recompiler: Testing memory operations...");
    
    // Test optimized memory allocation
    void* microProgram = allocateMicroProgram();
    void* codeBlock = allocateCodeBlock();
    
    if (microProgram && codeBlock) {
        Console.WriteLn("ARM64 Recompiler: Memory allocation successful");
        
        // Test fast memory operations
        uint8_t testData[1024];
        uint8_t copyBuffer[1024];
        
        // Fill with test pattern
        fastMemsetNEON(testData, 0xAA, sizeof(testData));
        
        // Copy using NEON
        fastMemcpyNEON(copyBuffer, testData, sizeof(testData));
        
        // Verify
        bool copyCorrect = true;
        for (size_t i = 0; i < sizeof(testData); ++i) {
            if (copyBuffer[i] != 0xAA) {
                copyCorrect = false;
                break;
            }
        }
        
        if (copyCorrect) {
            Console.WriteLn("ARM64 Recompiler: Memory operations test PASSED");
        } else {
            Console.Error("ARM64 Recompiler: Memory operations test FAILED");
        }
        
        // Clean up
        releaseMicroProgram(microProgram);
        releaseCodeBlock(codeBlock);
    } else {
        Console.Error("ARM64 Recompiler: Memory allocation failed");
    }
}

void testVectorOperations() {
    Console.WriteLn("ARM64 Recompiler: Testing vector operations...");
    
    float a[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float b[4] = {5.0f, 6.0f, 7.0f, 8.0f};
    float result[4];
    
    // Test vector addition
    vectorAdd4f_simple(result, a, b);
    bool addCorrect = (result[0] == 6.0f && result[1] == 8.0f && 
                      result[2] == 10.0f && result[3] == 12.0f);
    
    // Test vector multiplication
    vectorMul4f_simple(result, a, b);
    bool mulCorrect = (result[0] == 5.0f && result[1] == 12.0f && 
                      result[2] == 21.0f && result[3] == 32.0f);
    
    // Test dot product (3D)
    float dot = vectorDot3f_simple(a, b);
    bool dotCorrect = (dot == 38.0f); // 1*5 + 2*6 + 3*7 = 38
    
    if (addCorrect && mulCorrect && dotCorrect) {
        Console.WriteLn("ARM64 Recompiler: Vector operations test PASSED");
    } else {
        Console.Error("ARM64 Recompiler: Vector operations test FAILED");
    }
}

void testVIFUnpacking() {
    Console.WriteLn("ARM64 Recompiler: Testing VIF unpacking...");
    
    // Test V4-32 unpacking
    uint32_t src32[16] = {
        0x11111111, 0x22222222, 0x33333333, 0x44444444,
        0x55555555, 0x66666666, 0x77777777, 0x88888888,
        0x99999999, 0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC,
        0xDDDDDDDD, 0xEEEEEEEE, 0xFFFFFFFF, 0x00000000
    };
    uint32_t dst32[16];
    
    vifUnpackV4_32_simple(dst32, src32, 4);
    
    bool unpack32Correct = true;
    for (int i = 0; i < 16; i++) {
        if (dst32[i] != src32[i]) {
            unpack32Correct = false;
            break;
        }
    }
    
    // Test V4-16 unpacking
    uint16_t src16[16] = {
        0x1111, 0x2222, 0x3333, 0x4444,
        0x5555, 0x6666, 0x7777, 0x8888,
        0x9999, 0xAAAA, 0xBBBB, 0xCCCC,
        0xDDDD, 0xEEEE, 0xFFFF, 0x0000
    };
    uint32_t dst16[16];
    
    vifUnpackV4_16_simple(dst16, src16, 4);
    
    bool unpack16Correct = (dst16[0] == 0x1111 && dst16[1] == 0x2222 &&
                           dst16[2] == 0x3333 && dst16[3] == 0x4444);
    
    if (unpack32Correct && unpack16Correct) {
        Console.WriteLn("ARM64 Recompiler: VIF unpacking test PASSED");
    } else {
        Console.Error("ARM64 Recompiler: VIF unpacking test FAILED");
    }
}

void testMatrixOperations() {
    Console.WriteLn("ARM64 Recompiler: Testing matrix operations...");
    
    // Identity matrix
    float identity[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    
    // Test matrix
    float test[16] = {
        2.0f, 3.0f, 4.0f, 5.0f,
        6.0f, 7.0f, 8.0f, 9.0f,
        10.0f, 11.0f, 12.0f, 13.0f,
        14.0f, 15.0f, 16.0f, 17.0f
    };
    
    float result[16];
    
    // Multiply test matrix by identity (should equal test matrix)
    matrixMul4x4NEON(result, test, identity);
    
    bool matrixCorrect = true;
    for (int i = 0; i < 16; i++) {
        if (result[i] != test[i]) {
            matrixCorrect = false;
            break;
        }
    }
    
    if (matrixCorrect) {
        Console.WriteLn("ARM64 Recompiler: Matrix operations test PASSED");
    } else {
        Console.Error("ARM64 Recompiler: Matrix operations test FAILED");
    }
}

void testPerformanceCounters() {
    Console.WriteLn("ARM64 Recompiler: Testing performance counters...");
    
    uint64_t start = getPerformanceCounter();
    
    // Do some work
    volatile int sum = 0;
    for (int i = 0; i < 10000; i++) {
        sum += i;
    }
    
    uint64_t end = getPerformanceCounter();
    
    if (end > start) {
        Console.WriteLn("ARM64 Recompiler: Performance counter test PASSED");
        Console.WriteLn("ARM64 Recompiler: Cycles elapsed: %llu", end - start);
    } else {
        Console.Error("ARM64 Recompiler: Performance counter test FAILED");
    }
}

void testFeatureDetection() {
    Console.WriteLn("ARM64 Recompiler: Testing feature detection...");
    
    int neonAvailable = isNEONAvailable();
    uint32_t features = getARM64Features();
    
    Console.WriteLn("ARM64 Recompiler: NEON available: %s", neonAvailable ? "YES" : "NO");
    Console.WriteLn("ARM64 Recompiler: Feature flags: 0x%08X", features);
    
    if (neonAvailable) {
        Console.WriteLn("ARM64 Recompiler: Feature detection test PASSED");
    } else {
        Console.Error("ARM64 Recompiler: Feature detection test FAILED");
    }
}

void runAllTests() {
    Console.WriteLn("ARM64 Recompiler: Starting integration tests...");
    
    testMemoryOperations();
    testVectorOperations();
    testVIFUnpacking();
    testMatrixOperations();
    testPerformanceCounters();
    testFeatureDetection();
    
    Console.WriteLn("ARM64 Recompiler: Integration tests completed!");
}

} // namespace ARM64RecompilerTest

#else // !ARM64_RECOMPILER_ENABLED

namespace ARM64RecompilerTest {
void runAllTests() {
    // ARM64 Recompiler not enabled
}
}

#endif // ARM64_RECOMPILER_ENABLED
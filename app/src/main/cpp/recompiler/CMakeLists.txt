# ARM64 Static Recompiler Library - Simplified Version
cmake_minimum_required(VERSION 3.18)

project(libRecompiler VERSION 1.0 LANGUAGES CXX C)

# Target ARM64 architecture exclusively
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(ARM64_ONLY ON)

# Compiler settings for maximum performance
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ARM64-specific compiler flags for NEON optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a+simd")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffast-math")

# Simple recompiler sources (just the essential ones that compile)
set(RECOMPILER_SOURCES
    core/MemoryPool_Simple.cpp
    core/SimpleAPI.cpp
    arm64/NEON_SimpleOps.cpp
)

# Create static library
add_library(Recompiler STATIC ${RECOMPILER_SOURCES})

# Set output name and properties
set_target_properties(Recompiler PROPERTIES 
    OUTPUT_NAME "Recompiler"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# ARM64 feature detection and capabilities
target_compile_definitions(Recompiler PRIVATE
    ARM64_NEON_ENABLED=1
    PCSX2_RECOMPILER_OPTIMIZED=1
)

# Install library
install(TARGETS Recompiler
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "common/Pcsx2Types.h"

#ifdef _M_ARM64

namespace GSTextureNEON {

void ConvertRGBA8ToFloat32_NEON(const u8* src, float* dst, u32 count);
void ConvertFloat32ToRGBA8_NEON(const float* src, u8* dst, u32 count);
void BlendPixels_NEON(const u8* src1, const u8* src2, u8* dst, u32 count, float alpha);
void BilinearFilter_NEON(const u8* src, u8* dst, u32 src_width, u32 src_height, u32 dst_width, u32 dst_height);
void TextureSwizzle_NEON(const u8* src, u8* dst, u32 width, u32 height, u32 block_width, u32 block_height);
void MatrixMultiply4x4_NEON(const float* a, const float* b, float* result);
void Vector4Transform_NEON(const float* vec, const float* matrix, float* result, u32 count);
void ResetPerformanceCounters();

} // namespace GSTextureNEON

#endif // _M_ARM64

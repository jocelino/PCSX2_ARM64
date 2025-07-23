// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "GSTexture.h"

#ifdef _M_ARM64
#include <arm_neon.h>

namespace GSTextureNEON {

void ConvertRGBA8ToFloat32_NEON(const u8* src, float* dst, u32 count) {
	const u32 simd_count = count & ~7;
	
	for (u32 i = 0; i < simd_count; i += 8) {
		uint8x16_t rgba8_0 = vld1q_u8(&src[i * 4]);
		uint8x16_t rgba8_1 = vld1q_u8(&src[i * 4 + 16]);
		
		uint16x8_t rgba16_0 = vmovl_u8(vget_low_u8(rgba8_0));
		uint16x8_t rgba16_1 = vmovl_u8(vget_high_u8(rgba8_0));
		uint16x8_t rgba16_2 = vmovl_u8(vget_low_u8(rgba8_1));
		uint16x8_t rgba16_3 = vmovl_u8(vget_high_u8(rgba8_1));
		
		float32x4_t scale = vdupq_n_f32(1.0f / 255.0f);
		
		float32x4_t f0 = vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_low_u16(rgba16_0))), scale);
		float32x4_t f1 = vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(rgba16_0))), scale);
		float32x4_t f2 = vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_low_u16(rgba16_1))), scale);
		float32x4_t f3 = vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(rgba16_1))), scale);
		
		vst1q_f32(&dst[i * 4], f0);
		vst1q_f32(&dst[i * 4 + 4], f1);
		vst1q_f32(&dst[i * 4 + 8], f2);
		vst1q_f32(&dst[i * 4 + 12], f3);
	}
	
	for (u32 i = simd_count; i < count; i++) {
		dst[i * 4 + 0] = src[i * 4 + 0] / 255.0f;
		dst[i * 4 + 1] = src[i * 4 + 1] / 255.0f;
		dst[i * 4 + 2] = src[i * 4 + 2] / 255.0f;
		dst[i * 4 + 3] = src[i * 4 + 3] / 255.0f;
	}
}

void ConvertFloat32ToRGBA8_NEON(const float* src, u8* dst, u32 count) {
	const u32 simd_count = count & ~7;
	
	for (u32 i = 0; i < simd_count; i += 8) {
		float32x4_t scale = vdupq_n_f32(255.0f);
		
		float32x4_t f0 = vmulq_f32(vld1q_f32(&src[i * 4]), scale);
		float32x4_t f1 = vmulq_f32(vld1q_f32(&src[i * 4 + 4]), scale);
		float32x4_t f2 = vmulq_f32(vld1q_f32(&src[i * 4 + 8]), scale);
		float32x4_t f3 = vmulq_f32(vld1q_f32(&src[i * 4 + 12]), scale);
		
		uint32x4_t u32_0 = vcvtq_u32_f32(f0);
		uint32x4_t u32_1 = vcvtq_u32_f32(f1);
		uint32x4_t u32_2 = vcvtq_u32_f32(f2);
		uint32x4_t u32_3 = vcvtq_u32_f32(f3);
		
		uint16x8_t u16_0 = vcombine_u16(vmovn_u32(u32_0), vmovn_u32(u32_1));
		uint16x8_t u16_1 = vcombine_u16(vmovn_u32(u32_2), vmovn_u32(u32_3));
		
		uint8x16_t u8_result = vcombine_u8(vmovn_u16(u16_0), vmovn_u16(u16_1));
		
		vst1q_u8(&dst[i * 4], u8_result);
	}
	
	for (u32 i = simd_count; i < count; i++) {
		dst[i * 4 + 0] = static_cast<u8>(src[i * 4 + 0] * 255.0f);
		dst[i * 4 + 1] = static_cast<u8>(src[i * 4 + 1] * 255.0f);
		dst[i * 4 + 2] = static_cast<u8>(src[i * 4 + 2] * 255.0f);
		dst[i * 4 + 3] = static_cast<u8>(src[i * 4 + 3] * 255.0f);
	}
}

void BlendPixels_NEON(const u8* src1, const u8* src2, u8* dst, u32 count, float alpha) {
	const u32 simd_count = count & ~15;
	uint8x16_t alpha_vec = vdupq_n_u8(static_cast<u8>(alpha * 255.0f));
	uint8x16_t inv_alpha_vec = vdupq_n_u8(255 - static_cast<u8>(alpha * 255.0f));
	
	for (u32 i = 0; i < simd_count; i += 16) {
		uint8x16_t s1 = vld1q_u8(&src1[i]);
		uint8x16_t s2 = vld1q_u8(&src2[i]);
		
		uint16x8_t s1_lo = vmull_u8(vget_low_u8(s1), vget_low_u8(alpha_vec));
		uint16x8_t s1_hi = vmull_u8(vget_high_u8(s1), vget_high_u8(alpha_vec));
		
		uint16x8_t s2_lo = vmull_u8(vget_low_u8(s2), vget_low_u8(inv_alpha_vec));
		uint16x8_t s2_hi = vmull_u8(vget_high_u8(s2), vget_high_u8(inv_alpha_vec));
		
		uint16x8_t result_lo = vaddq_u16(s1_lo, s2_lo);
		uint16x8_t result_hi = vaddq_u16(s1_hi, s2_hi);
		
		uint8x16_t result = vcombine_u8(vshrn_n_u16(result_lo, 8), vshrn_n_u16(result_hi, 8));
		
		vst1q_u8(&dst[i], result);
	}
	
	for (u32 i = simd_count; i < count; i++) {
		dst[i] = static_cast<u8>((src1[i] * alpha + src2[i] * (1.0f - alpha)));
	}
}

} // namespace GSTextureNEON

#endif // _M_ARM64

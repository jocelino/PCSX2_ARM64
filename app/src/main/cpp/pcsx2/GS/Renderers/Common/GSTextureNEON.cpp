// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "GSTexture.h"

#ifdef _M_ARM64
#include <arm_neon.h>

namespace GSTextureNEON {

static struct {
    u64 texture_conversions;
    u64 blending_operations;
    u64 filtering_operations;
    u64 matrix_operations;
    
    void Reset() {
        texture_conversions = 0;
        blending_operations = 0;
        filtering_operations = 0;
        matrix_operations = 0;
    }
} s_neon_perf;

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
	
	s_neon_perf.blending_operations++;
}

void BilinearFilter_NEON(const u8* src, u8* dst, u32 src_width, u32 src_height, u32 dst_width, u32 dst_height) {
	const float x_ratio = static_cast<float>(src_width - 1) / dst_width;
	const float y_ratio = static_cast<float>(src_height - 1) / dst_height;
	
	for (u32 y = 0; y < dst_height; y++) {
		const float src_y = y * y_ratio;
		const u32 y1 = static_cast<u32>(src_y);
		const u32 y2 = std::min(y1 + 1, src_height - 1);
		const float y_diff = src_y - y1;
		
		for (u32 x = 0; x < dst_width; x += 4) {
			const u32 pixels_to_process = std::min(4u, dst_width - x);
			
			for (u32 i = 0; i < pixels_to_process; i++) {
				const float src_x = (x + i) * x_ratio;
				const u32 x1 = static_cast<u32>(src_x);
				const u32 x2 = std::min(x1 + 1, src_width - 1);
				const float x_diff = src_x - x1;
				
				const u8* p1 = &src[(y1 * src_width + x1) * 4];
				const u8* p2 = &src[(y1 * src_width + x2) * 4];
				const u8* p3 = &src[(y2 * src_width + x1) * 4];
				const u8* p4 = &src[(y2 * src_width + x2) * 4];
				
				uint8x8_t p1_vec = vld1_u8(p1);
				uint8x8_t p2_vec = vld1_u8(p2);
				uint8x8_t p3_vec = vld1_u8(p3);
				uint8x8_t p4_vec = vld1_u8(p4);
				
				uint16x8_t p1_16 = vmovl_u8(p1_vec);
				uint16x8_t p2_16 = vmovl_u8(p2_vec);
				uint16x8_t p3_16 = vmovl_u8(p3_vec);
				uint16x8_t p4_16 = vmovl_u8(p4_vec);
				
				uint16x8_t x_diff_vec = vdupq_n_u16(static_cast<u16>(x_diff * 256));
				uint16x8_t y_diff_vec = vdupq_n_u16(static_cast<u16>(y_diff * 256));
				uint16x8_t inv_x_diff_vec = vdupq_n_u16(256 - static_cast<u16>(x_diff * 256));
				uint16x8_t inv_y_diff_vec = vdupq_n_u16(256 - static_cast<u16>(y_diff * 256));
				
				uint16x8_t top = vaddq_u16(
					vmulq_u16(p1_16, inv_x_diff_vec),
					vmulq_u16(p2_16, x_diff_vec)
				);
				uint16x8_t bottom = vaddq_u16(
					vmulq_u16(p3_16, inv_x_diff_vec),
					vmulq_u16(p4_16, x_diff_vec)
				);
				
				uint16x8_t result = vaddq_u16(
					vmulq_u16(top, inv_y_diff_vec),
					vmulq_u16(bottom, y_diff_vec)
				);
				
				result = vshrq_n_u16(result, 16);
				uint8x8_t result_u8 = vmovn_u16(result);
				
				vst1_lane_u32(reinterpret_cast<u32*>(&dst[((y * dst_width) + x + i) * 4]), 
					vreinterpret_u32_u8(result_u8), 0);
			}
		}
	}
	
	s_neon_perf.filtering_operations++;
}

void TextureSwizzle_NEON(const u8* src, u8* dst, u32 width, u32 height, u32 block_width, u32 block_height) {
	const u32 blocks_x = width / block_width;
	const u32 blocks_y = height / block_height;
	
	for (u32 by = 0; by < blocks_y; by++) {
		for (u32 bx = 0; bx < blocks_x; bx++) {
			for (u32 y = 0; y < block_height; y++) {
				for (u32 x = 0; x < block_width; x += 16) {
					const u32 src_x = bx * block_width + x;
					const u32 src_y = by * block_height + y;
					const u32 dst_offset = ((by * blocks_x + bx) * block_width * block_height + y * block_width + x) * 4;
					const u32 src_offset = (src_y * width + src_x) * 4;
					
					if (x + 16 <= block_width) {
						uint8x16_t data1 = vld1q_u8(&src[src_offset]);
						uint8x16_t data2 = vld1q_u8(&src[src_offset + 16]);
						uint8x16_t data3 = vld1q_u8(&src[src_offset + 32]);
						uint8x16_t data4 = vld1q_u8(&src[src_offset + 48]);
						
						vst1q_u8(&dst[dst_offset], data1);
						vst1q_u8(&dst[dst_offset + 16], data2);
						vst1q_u8(&dst[dst_offset + 32], data3);
						vst1q_u8(&dst[dst_offset + 48], data4);
					} else {
						for (u32 i = 0; i < block_width - x; i++) {
							dst[dst_offset + i * 4] = src[src_offset + i * 4];
							dst[dst_offset + i * 4 + 1] = src[src_offset + i * 4 + 1];
							dst[dst_offset + i * 4 + 2] = src[src_offset + i * 4 + 2];
							dst[dst_offset + i * 4 + 3] = src[src_offset + i * 4 + 3];
						}
					}
				}
			}
		}
	}
	
	s_neon_perf.filtering_operations++;
}

void MatrixMultiply4x4_NEON(const float* a, const float* b, float* result) {
	float32x4_t a_row0 = vld1q_f32(&a[0]);
	float32x4_t a_row1 = vld1q_f32(&a[4]);
	float32x4_t a_row2 = vld1q_f32(&a[8]);
	float32x4_t a_row3 = vld1q_f32(&a[12]);
	
	for (int i = 0; i < 4; i++) {
		float32x4_t b_col = {b[i], b[i + 4], b[i + 8], b[i + 12]};
		
		float32x4_t result_col = vmulq_f32(a_row0, vdupq_laneq_f32(b_col, 0));
		result_col = vmlaq_f32(result_col, a_row1, vdupq_laneq_f32(b_col, 1));
		result_col = vmlaq_f32(result_col, a_row2, vdupq_laneq_f32(b_col, 2));
		result_col = vmlaq_f32(result_col, a_row3, vdupq_laneq_f32(b_col, 3));
		
		vst1q_f32(&result[i * 4], result_col);
	}
	
	s_neon_perf.matrix_operations++;
}

void Vector4Transform_NEON(const float* vec, const float* matrix, float* result, u32 count) {
	float32x4_t m_row0 = vld1q_f32(&matrix[0]);
	float32x4_t m_row1 = vld1q_f32(&matrix[4]);
	float32x4_t m_row2 = vld1q_f32(&matrix[8]);
	float32x4_t m_row3 = vld1q_f32(&matrix[12]);
	
	for (u32 i = 0; i < count; i++) {
		float32x4_t v = vld1q_f32(&vec[i * 4]);
		
		float32x4_t result_vec = vmulq_f32(m_row0, vdupq_laneq_f32(v, 0));
		result_vec = vmlaq_f32(result_vec, m_row1, vdupq_laneq_f32(v, 1));
		result_vec = vmlaq_f32(result_vec, m_row2, vdupq_laneq_f32(v, 2));
		result_vec = vmlaq_f32(result_vec, m_row3, vdupq_laneq_f32(v, 3));
		
		vst1q_f32(&result[i * 4], result_vec);
	}
	
	s_neon_perf.matrix_operations++;
}

void ResetPerformanceCounters() {
    s_neon_perf.Reset();
}

} // namespace GSTextureNEON

#endif // _M_ARM64

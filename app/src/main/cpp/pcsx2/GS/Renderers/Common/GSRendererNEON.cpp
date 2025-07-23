// SPDX-FileCopyrightText: 2002-2025 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "GSRenderer.h"

#ifdef _M_ARM64
#include <arm_neon.h>

namespace GSRendererNEON {

void OptimizeVertexProcessing_NEON(GSVertex* vertices, u32 count) {
	const u32 simd_count = count & ~3;
	
	for (u32 i = 0; i < simd_count; i += 4) {
		float32x4_t x_vals = {vertices[i].XYZ.X, vertices[i+1].XYZ.X, vertices[i+2].XYZ.X, vertices[i+3].XYZ.X};
		float32x4_t y_vals = {vertices[i].XYZ.Y, vertices[i+1].XYZ.Y, vertices[i+2].XYZ.Y, vertices[i+3].XYZ.Y};
		float32x4_t z_vals = {vertices[i].XYZ.Z, vertices[i+1].XYZ.Z, vertices[i+2].XYZ.Z, vertices[i+3].XYZ.Z};
		
		x_vals = vmaxq_f32(x_vals, vdupq_n_f32(-32768.0f));
		x_vals = vminq_f32(x_vals, vdupq_n_f32(32767.0f));
		y_vals = vmaxq_f32(y_vals, vdupq_n_f32(-32768.0f));
		y_vals = vminq_f32(y_vals, vdupq_n_f32(32767.0f));
		z_vals = vmaxq_f32(z_vals, vdupq_n_f32(0.0f));
		z_vals = vminq_f32(z_vals, vdupq_n_f32(65535.0f));
		
		vertices[i].XYZ.X = vgetq_lane_f32(x_vals, 0);
		vertices[i+1].XYZ.X = vgetq_lane_f32(x_vals, 1);
		vertices[i+2].XYZ.X = vgetq_lane_f32(x_vals, 2);
		vertices[i+3].XYZ.X = vgetq_lane_f32(x_vals, 3);
		
		vertices[i].XYZ.Y = vgetq_lane_f32(y_vals, 0);
		vertices[i+1].XYZ.Y = vgetq_lane_f32(y_vals, 1);
		vertices[i+2].XYZ.Y = vgetq_lane_f32(y_vals, 2);
		vertices[i+3].XYZ.Y = vgetq_lane_f32(y_vals, 3);
		
		vertices[i].XYZ.Z = vgetq_lane_f32(z_vals, 0);
		vertices[i+1].XYZ.Z = vgetq_lane_f32(z_vals, 1);
		vertices[i+2].XYZ.Z = vgetq_lane_f32(z_vals, 2);
		vertices[i+3].XYZ.Z = vgetq_lane_f32(z_vals, 3);
	}
	
	for (u32 i = simd_count; i < count; i++) {
		vertices[i].XYZ.X = std::clamp(vertices[i].XYZ.X, -32768.0f, 32767.0f);
		vertices[i].XYZ.Y = std::clamp(vertices[i].XYZ.Y, -32768.0f, 32767.0f);
		vertices[i].XYZ.Z = std::clamp(vertices[i].XYZ.Z, 0.0f, 65535.0f);
	}
}

void OptimizeTextureCoordinates_NEON(GSVertex* vertices, u32 count, float scale_u, float scale_v) {
	const u32 simd_count = count & ~3;
	float32x4_t scale_u_vec = vdupq_n_f32(scale_u);
	float32x4_t scale_v_vec = vdupq_n_f32(scale_v);
	
	for (u32 i = 0; i < simd_count; i += 4) {
		float32x4_t u_vals = {vertices[i].U, vertices[i+1].U, vertices[i+2].U, vertices[i+3].U};
		float32x4_t v_vals = {vertices[i].V, vertices[i+1].V, vertices[i+2].V, vertices[i+3].V};
		
		u_vals = vmulq_f32(u_vals, scale_u_vec);
		v_vals = vmulq_f32(v_vals, scale_v_vec);
		
		vertices[i].U = vgetq_lane_f32(u_vals, 0);
		vertices[i+1].U = vgetq_lane_f32(u_vals, 1);
		vertices[i+2].U = vgetq_lane_f32(u_vals, 2);
		vertices[i+3].U = vgetq_lane_f32(u_vals, 3);
		
		vertices[i].V = vgetq_lane_f32(v_vals, 0);
		vertices[i+1].V = vgetq_lane_f32(v_vals, 1);
		vertices[i+2].V = vgetq_lane_f32(v_vals, 2);
		vertices[i+3].V = vgetq_lane_f32(v_vals, 3);
	}
	
	for (u32 i = simd_count; i < count; i++) {
		vertices[i].U *= scale_u;
		vertices[i].V *= scale_v;
	}
}

void OptimizeColorProcessing_NEON(GSVertex* vertices, u32 count) {
	const u32 simd_count = count & ~3;
	
	for (u32 i = 0; i < simd_count; i += 4) {
		uint8x16_t colors = {
			vertices[i].RGBAQ.R, vertices[i].RGBAQ.G, vertices[i].RGBAQ.B, vertices[i].RGBAQ.A,
			vertices[i+1].RGBAQ.R, vertices[i+1].RGBAQ.G, vertices[i+1].RGBAQ.B, vertices[i+1].RGBAQ.A,
			vertices[i+2].RGBAQ.R, vertices[i+2].RGBAQ.G, vertices[i+2].RGBAQ.B, vertices[i+2].RGBAQ.A,
			vertices[i+3].RGBAQ.R, vertices[i+3].RGBAQ.G, vertices[i+3].RGBAQ.B, vertices[i+3].RGBAQ.A
		};
		
		uint16x8_t colors_16_lo = vmovl_u8(vget_low_u8(colors));
		uint16x8_t colors_16_hi = vmovl_u8(vget_high_u8(colors));
		
		float32x4_t colors_f32_0 = vcvtq_f32_u32(vmovl_u16(vget_low_u16(colors_16_lo)));
		float32x4_t colors_f32_1 = vcvtq_f32_u32(vmovl_u16(vget_high_u16(colors_16_lo)));
		float32x4_t colors_f32_2 = vcvtq_f32_u32(vmovl_u16(vget_low_u16(colors_16_hi)));
		float32x4_t colors_f32_3 = vcvtq_f32_u32(vmovl_u16(vget_high_u16(colors_16_hi)));
		
		float32x4_t scale = vdupq_n_f32(1.0f / 255.0f);
		colors_f32_0 = vmulq_f32(colors_f32_0, scale);
		colors_f32_1 = vmulq_f32(colors_f32_1, scale);
		colors_f32_2 = vmulq_f32(colors_f32_2, scale);
		colors_f32_3 = vmulq_f32(colors_f32_3, scale);
		
		uint32x4_t colors_u32_0 = vcvtq_u32_f32(vmulq_f32(colors_f32_0, vdupq_n_f32(255.0f)));
		uint32x4_t colors_u32_1 = vcvtq_u32_f32(vmulq_f32(colors_f32_1, vdupq_n_f32(255.0f)));
		uint32x4_t colors_u32_2 = vcvtq_u32_f32(vmulq_f32(colors_f32_2, vdupq_n_f32(255.0f)));
		uint32x4_t colors_u32_3 = vcvtq_u32_f32(vmulq_f32(colors_f32_3, vdupq_n_f32(255.0f)));
		
		uint16x8_t colors_u16_lo = vcombine_u16(vmovn_u32(colors_u32_0), vmovn_u32(colors_u32_1));
		uint16x8_t colors_u16_hi = vcombine_u16(vmovn_u32(colors_u32_2), vmovn_u32(colors_u32_3));
		uint8x16_t result = vcombine_u8(vmovn_u16(colors_u16_lo), vmovn_u16(colors_u16_hi));
		
		vertices[i].RGBAQ.R = vgetq_lane_u8(result, 0);
		vertices[i].RGBAQ.G = vgetq_lane_u8(result, 1);
		vertices[i].RGBAQ.B = vgetq_lane_u8(result, 2);
		vertices[i].RGBAQ.A = vgetq_lane_u8(result, 3);
		
		vertices[i+1].RGBAQ.R = vgetq_lane_u8(result, 4);
		vertices[i+1].RGBAQ.G = vgetq_lane_u8(result, 5);
		vertices[i+1].RGBAQ.B = vgetq_lane_u8(result, 6);
		vertices[i+1].RGBAQ.A = vgetq_lane_u8(result, 7);
		
		vertices[i+2].RGBAQ.R = vgetq_lane_u8(result, 8);
		vertices[i+2].RGBAQ.G = vgetq_lane_u8(result, 9);
		vertices[i+2].RGBAQ.B = vgetq_lane_u8(result, 10);
		vertices[i+2].RGBAQ.A = vgetq_lane_u8(result, 11);
		
		vertices[i+3].RGBAQ.R = vgetq_lane_u8(result, 12);
		vertices[i+3].RGBAQ.G = vgetq_lane_u8(result, 13);
		vertices[i+3].RGBAQ.B = vgetq_lane_u8(result, 14);
		vertices[i+3].RGBAQ.A = vgetq_lane_u8(result, 15);
	}
}

} // namespace GSRendererNEON

#endif // _M_ARM64

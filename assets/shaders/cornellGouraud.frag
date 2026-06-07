#version 450
/*
 * Copyright 2023 TU Wien, Institute of Visual Computing & Human-Centered Technology.
 * This file is part of the GCG Lab Framework and must not be redistributed.
 *
 * Original version created by Lukas Gersthofer and Bernhard Steiner.
 * Vulkan edition created by Johannes Unterguggenberger (junt@cg.tuwien.ac.at).
 */
 
layout (location = 0) in VertexData {
	vec3 color;
	vec3 positionLocal;
	vec3 normalLocal;
} frag_in;

layout (binding = 3) uniform sampler2D diffuse_texture;

layout (location = 0) out vec4 out_color;

void main() {	
	vec3 color = frag_in.color;

	// Texture only the Cornell floor; keep walls/ceiling in vertex colors.
	if (frag_in.normalLocal.y > 0.9) {
		vec2 floor_uv = frag_in.positionLocal.xz * 0.65;
		vec3 floor_tex = texture(diffuse_texture, floor_uv).rgb;
		color *= floor_tex;
	}

	out_color = vec4(color, 1.0);
}

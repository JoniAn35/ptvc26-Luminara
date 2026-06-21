#version 450
/*
 * Copyright 2023 TU Wien, Institute of Visual Computing & Human-Centered Technology.
 * This file is part of the GCG Lab Framework and must not be redistributed.
 *
 * Original version created by Lukas Gersthofer and Bernhard Steiner.
 * Vulkan edition created by Johannes Unterguggenberger (junt@cg.tuwien.ac.at).
 */
 
layout (binding = 0) uniform UniformBuffer {
	vec4 color;
	mat4 modelMatrix;
	mat4 modelMatrixForNormals;
	mat4 viewProjMatrix;
	vec4 cameraPosition;
	vec4 illumination; // ka, kd, ks, alpha
	ivec4 userInput;
	mat4 reflectionViewProjMatrix;
} ub_data;

layout (binding = 1) uniform DirectionalLight {
	vec4 color;
	vec4 direction;
} dl_data;

layout (binding = 2) uniform PointLight {
	vec4 color;
	vec4 position;
	vec4 attenuation;
} pl_data;

layout (binding = 3) uniform sampler2D diffuse_texture;
layout (binding = 4) uniform samplerCube environment_cubemap;

layout (location = 0) in VertexData {
	vec3 positionWorld;
	vec3 normalWorld;
	vec2 textureCoordinates;
} frag_in;

layout (location = 0) out vec4 out_color;

vec3 phong(vec3 n, vec3 l, vec3 v, vec3 diffuseC, float diffuseF, vec3 specularC, float specularF, float alpha, bool attenuate, vec3 attenuation) {
	float d = length(l);
	l = normalize(l);
	float att = 1.0;	
	if(attenuate) {
		att = 1.0f / (attenuation.x + d * attenuation.y + d * d * attenuation.z);
	}
	vec3 r = reflect(-l, n);
	return (diffuseF * diffuseC * max(0, dot(n, l)) + specularF * specularC * pow(max(0, dot(r, v)), alpha)) * att; 
}

// Gets the reflected color value from a a certain position, from 
// a certain direction INSIDE of a cornell box of size 3 which is 
// positioned at the origin.
// positionWS:  Position inside the cornell box for which to get the
//              reflected color value for from -directionWS.
// directionWS: Outgoing direction vector (from positionWS towards the
//              outside) for which to get the reflected color value for.
vec3 getCornellBoxReflectionColor(vec3 positionWS, vec3 directionWS) {
	vec3 P0 = positionWS;
	vec3 V  = normalize(directionWS);

	const float boxSize = 1.5;
	vec4[5] planes = {
		vec4(-1.0,  0.0,  0.0, -boxSize), // left
		vec4( 1.0,  0.0,  0.0, -boxSize), // right
		vec4( 0.0,  1.0,  0.0, -boxSize), // top
		vec4( 0.0, -1.0,  0.0, -boxSize), // bottom
		vec4( 0.0,  0.0, -1.0, -boxSize)  // back
	};
	vec3[5] colors = {
		vec3(0.49, 0.06, 0.22),    // left
		vec3(0.0, 0.13, 0.31),    // right
		vec3(0.96, 0.93, 0.85), // top
		vec3(0.64, 0.64, 0.64), // bottom
		vec3(0.76, 0.74, 0.68)  // back
	};

	for (int i = 0; i < 5; ++i) {
		vec3  N = planes[i].xyz;
		float d = planes[i].w;
		float denom = dot(V, N);
		if (denom <= 0) continue;
		float t = -(dot(P0, N) + d)/denom;
		vec3  P = P0 + t*V;
		float q = boxSize + 0.01;
		if (P.x > -q && P.x < q && P.y > -q && P.y < q && P.z > -q && P.z < q) {
			return colors[i];
		}
	}
	return vec3(0.0, 0.0, 0.0);
}

// Computes the reflection direction for an incident vector I about normal N, 
// and clamps the reflection to a maximum of 180, i.e. the reflection vector
// will always lie within the hemisphere around normal N.
// Aside from clamping, this function produces the same result as GLSL's reflect function.
vec3 clampedReflect(vec3 I, vec3 N)
{
	return I - 2.0 * min(dot(N, I), 0.0) * N;
}

vec3 fresnelSchlick (float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow (1.0 - cosTheta, 5.0);
}

void main() {	
	const vec3 beamColor = vec3(179.0 / 255.0, 28.0 / 255.0, 28.0 / 255.0);
	vec3 n = normalize(frag_in.normalWorld);
	if (ub_data.userInput[3] == 2) {
		// Sensor is a hemisphere: discard the half that would lie behind the right wall.
		float sensorCenterX = ub_data.modelMatrix[3].x;
		if (frag_in.positionWorld.x > sensorCenterX) {
			discard;
		}
	}
	if (ub_data.userInput[3] == 6) {
		// Beam glow shell with hard core and soft edge fade.
		vec3 viewDir = normalize(ub_data.cameraPosition.xyz - frag_in.positionWorld);
		float facing = clamp(abs(dot(n, viewDir)), 0.0, 1.0);

		// Sharp center plus wider halo.
		float hardCore = pow(facing, 7.0);
		float softHalo = pow(facing, 1.35);

		vec3 glowColor = beamColor * (0.95 + 2.40 * hardCore + 0.90 * softHalo);
		float glowAlpha = ub_data.illumination.a * (1.25 * hardCore + 0.75 * softHalo);
		glowAlpha = clamp(glowAlpha, 0.0, 1.0);
		out_color = vec4(glowColor, glowAlpha);
		return;
	}
	if (ub_data.userInput[3] == 7) {
		// Bloom composite: blur bright samples from a separate render target.
		vec2 uv = clamp(frag_in.textureCoordinates, vec2(0.001), vec2(0.999));
		vec2 texel = 1.0 / vec2(textureSize(diffuse_texture, 0));

		float threshold = 0.82;
		float radius = 1.75;
		float intensity = 1.05;

		vec2 offsets[13] = vec2[](
			vec2( 0.0,  0.0),
			vec2( 1.0,  0.0), vec2(-1.0,  0.0),
			vec2( 0.0,  1.0), vec2( 0.0, -1.0),
			vec2( 1.0,  1.0), vec2(-1.0,  1.0),
			vec2( 1.0, -1.0), vec2(-1.0, -1.0),
			vec2( 2.0,  0.0), vec2(-2.0,  0.0),
			vec2( 0.0,  2.0), vec2( 0.0, -2.0)
		);
		float weights[13] = float[](
			0.20,
			0.11, 0.11,
			0.11, 0.11,
			0.08, 0.08,
			0.08, 0.08,
			0.04, 0.04,
			0.04, 0.04
		);

		vec3 bloom = vec3(0.0);
		for (int i = 0; i < 13; ++i) {
			vec2 sampleUv = clamp(uv + offsets[i] * texel * radius, vec2(0.0), vec2(1.0));
			vec3 sampleColor = texture(diffuse_texture, sampleUv).rgb;
			float luminance = dot(sampleColor, vec3(0.2126, 0.7152, 0.0722));

			// Limit bloom to red-dominant highlights to avoid wall/ceiling bleed.
			float redDominance = sampleColor.r - max(sampleColor.g, sampleColor.b);
			float redMask = smoothstep(0.06, 0.20, redDominance);
			float bright = clamp((luminance - threshold) / max(1.0 - threshold, 1e-4), 0.0, 1.0) * redMask;
			bloom += sampleColor * bright * weights[i];
		}

		bloom *= intensity;
		float bloomAlpha = clamp(max(max(bloom.r, bloom.g), bloom.b) * ub_data.illumination.a, 0.0, 1.0);
		out_color = vec4(bloom, bloomAlpha);
		return;
	}
	vec3 v = normalize(frag_in.positionWorld - ub_data.cameraPosition.xyz);
	vec3 R = normalize(clampedReflect(v, n));
	vec3 reflectionColor = getCornellBoxReflectionColor(frag_in.positionWorld, R);
	if (ub_data.userInput[3] == 1) {
		// Project the current mirror fragment into the mirrored camera render target.
		vec4 clip = ub_data.reflectionViewProjMatrix * vec4(frag_in.positionWorld, 1.0);
		vec2 finalUv = clip.xy / max(clip.w, 1e-5);
		finalUv = finalUv * 0.5 + 0.5;
		// Widen mirror coverage to avoid the "zoomed-in" look on the mirror surface.
		finalUv = (finalUv - vec2(0.5)) * 1.5 + vec2(0.5);
		finalUv = clamp(finalUv, vec2(0.001), vec2(0.999));
		reflectionColor = texture(diffuse_texture, finalUv).rgb;
	}
	vec3 F0 = vec3(0.1); // default non-mirror base reflectivity
	vec3 reflectivity = fresnelSchlick(dot(n, -v), F0);
	float reflectionMask = 1.0;
	vec3 diffuseColor = vec3(0.82, 0.82, 0.82);
	if (ub_data.userInput[3] == 4) {
		diffuseColor = vec3(0.20, 0.20, 0.22);
		reflectionMask = 0.0;
	}
	if (ub_data.userInput[3] == 3) {
		diffuseColor = beamColor;
	}
	if (ub_data.userInput[3] == 2 && ub_data.illumination.y > 0.7) {
		diffuseColor = beamColor;
	}
	if (ub_data.userInput[3] == 1) {
		// Mirror front face is local +X after transform.
		vec3 mirrorFrontNormalWS = normalize((ub_data.modelMatrixForNormals * vec4(1.0, 0.0, 0.0, 0.0)).xyz);
		float frontAlignment = dot(n, mirrorFrontNormalWS);
		// Reflect only the front face.
		reflectionMask = frontAlignment > 0.8 ? 1.0 : 0.0;
		F0 = vec3(0.92);
		reflectivity = fresnelSchlick(dot(n, -v), F0);
		diffuseColor = vec3(0.03, 0.03, 0.04);
		if (frontAlignment < -0.8) {
			diffuseColor = vec3(0.05, 0.05, 0.06);
		}
	}

	// 2D UI overlay path uses materialProperties directly.
	if (ub_data.userInput[3] == 5) {
		float a = clamp(ub_data.illumination.a, 0.0, 1.0);
		if (a < 0.999) {
			// Ordered 4x4 Bayer dither to emulate transparency without alpha blending.
			int x = int(mod(gl_FragCoord.x, 4.0));
			int y = int(mod(gl_FragCoord.y, 4.0));
			float bayer[16] = float[16](
				0.0,  8.0,  2.0, 10.0,
				12.0, 4.0, 14.0,  6.0,
				3.0, 11.0,  1.0,  9.0,
				15.0, 7.0, 13.0,  5.0
			);
			float threshold = (bayer[y * 4 + x] + 0.5) / 16.0;
			if (a < threshold) {
				discard;
			}
		}
		out_color = vec4(ub_data.illumination.rgb, 1.0);
		return;
	}
	
	// Start with ambient illumination contribution:
	vec3 color = diffuseColor * ub_data.illumination[0];
	
	// Add directional light's contribution:
	color += phong(
		n, 
		-dl_data.direction.xyz, 
		-v, 
		dl_data.color.rgb * diffuseColor, ub_data.illumination[1], 
		dl_data.color.rgb,                ub_data.illumination[2], 
		ub_data.illumination[3], 
		false, vec3(1.0)
	);
			
	// Add point light's contribution
	color += phong(
		n, 
		pl_data.position.xyz - frag_in.positionWorld, 
		-v, 
		pl_data.color.rgb * diffuseColor, ub_data.illumination[1], 
		pl_data.color.rgb,                ub_data.illumination[2], 
		ub_data.illumination[3], 
		true, pl_data.attenuation.xyz
	);

	// Write color for the current fragment:
	out_color = vec4(color, 1.0);
	color = mix(color, reflectionColor, reflectivity * reflectionMask);
	out_color = vec4(color, 1.0);
}


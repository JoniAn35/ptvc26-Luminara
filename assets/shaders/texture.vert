#version 450
/*
 * Copyright 2023 TU Wien, Institute of Visual Computing & Human-Centered Technology.
 * This file is part of the GCG Lab Framework and must not be redistributed.
 *
 * Original version created by Lukas Gersthofer and Bernhard Steiner.
 * Vulkan edition created by Johannes Unterguggenberger (junt@cg.tuwien.ac.at).
 */

layout (location = 0) in vec3 in_position;
layout (location = 1) in vec3 in_normal;
layout (location = 2) in vec2 in_texture_coordinates;

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

layout (location = 0) out VertexData {
    vec3 positionWorld;
    vec3 normalWorld;
    vec2 textureCoordinates;
} vert_out;

void main() {
    vec3 pos = in_position;
    vec3 nrm = in_normal;

    // Pulsating traveling-wave animation for the light beam (modes 3 and 6).
    // Vertices are displaced radially, with a sine wave that travels along the
    // cylinder axis over time. Normals are re-derived analytically.
    if (ub_data.userInput[3] == 3 || ub_data.userInput[3] == 6) {
        float r = length(in_position.xz);
        if (r > 0.01) {
            float time    = ub_data.color.w;  // current time passed from CPU
            float A       = 0.10;             // radial amplitude as fraction of radius
            float omega_t = 9.0;              // temporal frequency (rad/s)
            float omega_y = 14.0;             // spatial frequency along cylinder axis
            float phase   = omega_t * time + omega_y * in_position.y;

            // Scale XZ radially: r' = r * (1 + A*sin(phase))
            float scale_r = 1.0 + A * sin(phase);
            pos = vec3(in_position.x * scale_r, in_position.y, in_position.z * scale_r);

            // Analytical outward normal for the deformed cylinder:
            // N'_local = normalize( (N.x, -r * dF/dy, N.z) )
            // where dF/dy = A * omega_y * cos(phase)
            float df_dy = A * omega_y * cos(phase);
            nrm = normalize(vec3(in_normal.x, -r * df_dy, in_normal.z));
        }
    }

    vec4 position_world = ub_data.modelMatrix * vec4(pos, 1.0);
    vert_out.positionWorld = position_world.xyz;
    gl_Position = ub_data.viewProjMatrix * position_world;

    vert_out.normalWorld = mat3(ub_data.modelMatrixForNormals) * nrm;
    vert_out.textureCoordinates = in_texture_coordinates;
}
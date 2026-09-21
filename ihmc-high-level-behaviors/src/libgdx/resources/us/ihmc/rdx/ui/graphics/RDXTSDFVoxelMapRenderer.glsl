#type vertex
#version 410

// xyz = world-space voxel center, w = TSDF value in [-1, 1].  Same for all 36 verts of one cube.
layout(location = 0) in vec4 a_positionAndTSDF;

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform mat3 u_mapRotation; // rotation from map frame to world frame
uniform float u_voxelSize;

// Unit cube (half-extent 0.5) — 6 faces × 2 triangles × 3 vertices, CCW winding viewed from outside.
const vec3 CUBE_VERTS[36] = vec3[36](
    // +X face
    vec3( 0.5, -0.5, -0.5), vec3( 0.5,  0.5, -0.5), vec3( 0.5,  0.5,  0.5),
    vec3( 0.5, -0.5, -0.5), vec3( 0.5,  0.5,  0.5), vec3( 0.5, -0.5,  0.5),
    // -X face
    vec3(-0.5,  0.5, -0.5), vec3(-0.5, -0.5, -0.5), vec3(-0.5, -0.5,  0.5),
    vec3(-0.5,  0.5, -0.5), vec3(-0.5, -0.5,  0.5), vec3(-0.5,  0.5,  0.5),
    // +Y face
    vec3(-0.5,  0.5, -0.5), vec3(-0.5,  0.5,  0.5), vec3( 0.5,  0.5, -0.5),
    vec3(-0.5,  0.5,  0.5), vec3( 0.5,  0.5,  0.5), vec3( 0.5,  0.5, -0.5),
    // -Y face
    vec3(-0.5, -0.5, -0.5), vec3( 0.5, -0.5, -0.5), vec3(-0.5, -0.5,  0.5),
    vec3( 0.5, -0.5, -0.5), vec3( 0.5, -0.5,  0.5), vec3(-0.5, -0.5,  0.5),
    // +Z face
    vec3(-0.5, -0.5,  0.5), vec3( 0.5, -0.5,  0.5), vec3( 0.5,  0.5,  0.5),
    vec3(-0.5, -0.5,  0.5), vec3( 0.5,  0.5,  0.5), vec3(-0.5,  0.5,  0.5),
    // -Z face
    vec3( 0.5, -0.5, -0.5), vec3(-0.5, -0.5, -0.5), vec3(-0.5,  0.5, -0.5),
    vec3( 0.5, -0.5, -0.5), vec3(-0.5,  0.5, -0.5), vec3( 0.5,  0.5, -0.5)
);

const vec3 FACE_NORMALS[6] = vec3[6](
    vec3( 1.0,  0.0,  0.0),
    vec3(-1.0,  0.0,  0.0),
    vec3( 0.0,  1.0,  0.0),
    vec3( 0.0, -1.0,  0.0),
    vec3( 0.0,  0.0,  1.0),
    vec3( 0.0,  0.0, -1.0)
);

void main()
{
    int vertInCube = gl_VertexID % 36;

    vec3 center = a_positionAndTSDF.xyz;
    float tsdf  = a_positionAndTSDF.w;

    vec3 cornerOffset = u_mapRotation * CUBE_VERTS[vertInCube] * u_voxelSize;
    vec3 worldPos = center + cornerOffset;

    gl_Position = u_projTrans * u_viewTrans * vec4(worldPos, 1.0);

    // Simple diffuse + ambient shading so cube faces are visually distinct.
    vec3 worldNormal = u_mapRotation * FACE_NORMALS[vertInCube / 6];
    vec3 sunDir = normalize(vec3(0.4, 0.3, 1.0));
    float diffuse = max(dot(worldNormal, sunDir), 0.0);
    float brightness = 0.35 + 0.65 * diffuse;

    // Color converges to white at tsdf=0 and diverges to blue (positive) or red (negative).
    // This avoids flickering at the surface where tsdf sign is noisy near zero.
    vec3 baseColor;
    if (tsdf >= 0.0)
        baseColor = mix(vec3(1.0, 1.0, 1.0), vec3(0.0, 0.0, 1.0), tsdf);
    else
        baseColor = mix(vec3(1.0, 1.0, 1.0), vec3(1.0, 0.0, 0.0), -tsdf);

    // Alpha: peaks at 1.0 at the surface (tsdf=0), falls to 0 at ±1.
    float alpha = 1.0 - abs(tsdf);

    v_color = vec4(baseColor * brightness, alpha);
}

#type fragment
#version 410

in vec4 v_color;
out vec4 color;

void main()
{
    color = v_color;
}

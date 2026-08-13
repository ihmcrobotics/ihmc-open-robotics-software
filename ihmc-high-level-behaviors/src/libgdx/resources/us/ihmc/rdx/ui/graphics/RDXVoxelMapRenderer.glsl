#type vertex
#version 410

layout(location = 0) in vec3 a_position; // world-space voxel center, same value for all 36 verts of a cube

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform mat3 u_mapRotation; // rotation from map frame to world frame
uniform float u_voxelSize;

// Unit cube (half-extent 0.5) — 6 faces × 2 triangles × 3 vertices, CCW winding viewed from outside.
// The vertex shader selects the corner for this invocation via gl_VertexID % 36.
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

// One outward normal per face group (each group = 6 consecutive verts).
const vec3 FACE_NORMALS[6] = vec3[6](
    vec3( 1.0,  0.0,  0.0),
    vec3(-1.0,  0.0,  0.0),
    vec3( 0.0,  1.0,  0.0),
    vec3( 0.0, -1.0,  0.0),
    vec3( 0.0,  0.0,  1.0),
    vec3( 0.0,  0.0, -1.0)
);

float linearInterpolate(float a, float b, float alpha)
{
    return (1.0f - alpha) * a + alpha * b;
}

vec4 colorFromHeight(float height)
{
    float magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
    float orangeR  = 1.0, orangeG  = 200.0 / 255.0, orangeB  = 0.0;
    float yellowR  = 1.0, yellowG  = 1.0, yellowB  = 0.0;
    float blueR    = 0.0, blueG    = 0.0, blueB    = 1.0;
    float greenR   = 0.0, greenG   = 1.0, greenB   = 0.0;

    float gradientSize   = 0.2;
    float gradientLength = 1.0;
    float alpha = mod(height, gradientLength);
    if (alpha < 0.0)
        alpha = 1.0 + alpha;
    while (alpha > 5.0 * gradientSize)
        alpha -= 5.0 * gradientSize;

    float r = 0.0, g = 0.0, b = 0.0;
    if (alpha <= gradientSize)
    {
        r = linearInterpolate(magentaR, blueR, alpha / gradientSize);
        g = linearInterpolate(magentaG, blueG, alpha / gradientSize);
        b = linearInterpolate(magentaB, blueB, alpha / gradientSize);
    }
    else if (alpha <= gradientSize * 2.0)
    {
        r = linearInterpolate(blueR, greenR, (alpha - gradientSize) / gradientSize);
        g = linearInterpolate(blueG, greenG, (alpha - gradientSize) / gradientSize);
        b = linearInterpolate(blueB, greenB, (alpha - gradientSize) / gradientSize);
    }
    else if (alpha <= gradientSize * 3.0)
    {
        r = linearInterpolate(greenR, yellowR, (alpha - 2.0 * gradientSize) / gradientSize);
        g = linearInterpolate(greenG, yellowG, (alpha - 2.0 * gradientSize) / gradientSize);
        b = linearInterpolate(greenB, yellowB, (alpha - 2.0 * gradientSize) / gradientSize);
    }
    else if (alpha <= gradientSize * 4.0)
    {
        r = linearInterpolate(yellowR, orangeR, (alpha - 3.0 * gradientSize) / gradientSize);
        g = linearInterpolate(yellowG, orangeG, (alpha - 3.0 * gradientSize) / gradientSize);
        b = linearInterpolate(yellowB, orangeB, (alpha - 3.0 * gradientSize) / gradientSize);
    }
    else if (alpha <= gradientSize * 5.0)
    {
        r = linearInterpolate(orangeR, magentaR, (alpha - 4.0 * gradientSize) / gradientSize);
        g = linearInterpolate(orangeG, magentaG, (alpha - 4.0 * gradientSize) / gradientSize);
        b = linearInterpolate(orangeB, magentaB, (alpha - 4.0 * gradientSize) / gradientSize);
    }

    return vec4(r, g, b, 1.0);
}

void main()
{
    int vertInCube = gl_VertexID % 36;

    // Rotate the unit-cube corner from map frame to world frame, then scale and offset from center
    vec3 cornerOffset = u_mapRotation * CUBE_VERTS[vertInCube] * u_voxelSize;
    vec3 worldPos = a_position + cornerOffset;

    gl_Position = u_projTrans * u_viewTrans * vec4(worldPos, 1.0);

    // Simple diffuse + ambient shading so cube faces are visually distinct
    vec3 worldNormal = u_mapRotation * FACE_NORMALS[vertInCube / 6];
    vec3 sunDir = normalize(vec3(0.4, 0.3, 1.0));
    float diffuse = max(dot(worldNormal, sunDir), 0.0);
    float brightness = 0.35 + 0.65 * diffuse;

    vec4 heightColor = colorFromHeight(worldPos.z);
    v_color = vec4(heightColor.rgb * brightness, 1.0);
}

#type fragment
#version 410

in vec4 v_color;
out vec4 color;

void main()
{
    color = v_color;
}

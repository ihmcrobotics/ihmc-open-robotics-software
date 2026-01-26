#type vertex
#version 410

layout(location = 0) in vec3 a_cubePos;
layout(location = 1) in vec3 a_voxelCenter;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_voxelSize;

out vec4 v_color;

void main()
{
    // Local cube vertex scaled to voxel size
    vec3 worldPos = a_voxelCenter + a_cubePos * u_voxelSize;

    vec4 cam = u_viewTrans * vec4(worldPos, 1.0);
    gl_Position = u_projTrans * cam;

    // Simple color: cyan for occupied voxels
    v_color = vec4(0.2, 0.8, 1.0, 1.0);
}

#type fragment
#version 410

in vec4 v_color;
out vec4 color;

void main()
{
    color = v_color;
}

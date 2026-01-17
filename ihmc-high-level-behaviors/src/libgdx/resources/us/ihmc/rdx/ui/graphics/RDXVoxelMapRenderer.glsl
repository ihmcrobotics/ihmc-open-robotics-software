#type vertex
#version 410

layout(location = 0) in vec3 a_position;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_voxelSize;
uniform float u_screenWidth;

out vec4 v_color;

void main()
{
    // Transform voxel position to camera space
    vec4 cam = u_viewTrans * vec4(a_position, 1.0);
    gl_Position = u_projTrans * cam;

    // Compute screen-space point size similar to height map
    vec4 p0 = u_projTrans * vec4(0.0, 0.0, cam.z, cam.w);
    vec4 p1 = u_projTrans * vec4(u_voxelSize, u_voxelSize, cam.z, cam.w);
    float projectedSize = u_screenWidth * p1.x / p1.w;

    gl_PointSize = max(1.0, 0.5 * projectedSize);

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

#type vertex
#version 410

layout(location = 0) in vec3 a_position;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_voxelSize;

out vec4 v_color;

#define CUBE_VERTEX_COUNT 36

const vec3 CUBE_VERTS[CUBE_VERTEX_COUNT] = vec3[CUBE_VERTEX_COUNT](
    // +X face (normal +X), view from +X looking at origin
    // corners on this face:
    // p0 = (0.5,  0.5,  0.5)   // top-front
    // p1 = (0.5, -0.5,  0.5)   // bottom-front
    // p2 = (0.5, -0.5, -0.5)   // bottom-back
    // p3 = (0.5,  0.5, -0.5)   // top-back

    // t0: p0, p1, p2
    vec3(0.5,  0.5,  0.5),  // p0
    vec3(0.5, -0.5,  0.5),  // p1
    vec3(0.5, -0.5, -0.5),  // p2

     // t1: p3, p0, p2
     vec3(0.5,  0.5, -0.5),  // p3
     vec3(0.5,  0.5,  0.5),  // p0
     vec3(0.5, -0.5, -0.5),  // p2

    // -X face (normal -X), view from -X looking at origin
    // p0 = (-0.5, -0.5, -0.5)  // bottom-back
    // p1 = (-0.5,  0.5, -0.5)  // top-back
    // p2 = (-0.5,  0.5,  0.5)  // top-front
    // p3 = (-0.5, -0.5,  0.5)  // bottom-front

    // t0: p0, p1, p2
    vec3(-0.5,  0.5, -0.5), // p1
    vec3(-0.5, -0.5, -0.5), // p0
    vec3(-0.5,  0.5,  0.5), // p2

    // t1: p0, p2, p3
    vec3(-0.5,  0.5,  0.5), // p2
    vec3(-0.5, -0.5, -0.5), // p0
    vec3(-0.5, -0.5,  0.5), // p3


    // +Y face (normal +Y), view from +Y looking at origin
    // p0 = (-0.5,  0.5, -0.5)  // left-back
    // p1 = (-0.5,  0.5,  0.5)  // left-front
    // p2 = ( 0.5,  0.5,  0.5)  // right-front
    // p3 = ( 0.5,  0.5, -0.5)  // right-back

    // t0: p0, p1, p2
    vec3(-0.5,  0.5, -0.5), // p0
    vec3(-0.5,  0.5,  0.5), // p1
    vec3(0.5,  0.5,  0.5),  // p2

    // t1: p0, p2, p3
    vec3(-0.5,  0.5, -0.5), // p0
    vec3(0.5,  0.5,  0.5),  // p2
    vec3(0.5,  0.5, -0.5),  // p3


    // -Y face (normal -Y), view from -Y looking at origin
    // p0 = (-0.5, -0.5, -0.5)  // left-back
    // p1 = ( 0.5, -0.5, -0.5)  // right-back
    // p2 = ( 0.5, -0.5,  0.5)  // right-front
    // p3 = (-0.5, -0.5,  0.5)  // left-front

    // t0: p0, p1, p2
    vec3(-0.5, -0.5, -0.5), // p0
    vec3(0.5, -0.5, -0.5),  // p1
    vec3(0.5, -0.5,  0.5),  // p2

    // t1: p0, p2, p3
    vec3(-0.5, -0.5, -0.5), // p0
    vec3(0.5, -0.5,  0.5),  // p2
    vec3(-0.5, -0.5,  0.5), // p3


    // +Z face (normal +Z), view from +Z looking at origin
    // p0 = (-0.5, -0.5,  0.5)  // bottom-left
    // p1 = ( 0.5, -0.5,  0.5)  // bottom-right
    // p2 = ( 0.5,  0.5,  0.5)  // top-right
    // p3 = (-0.5,  0.5,  0.5)  // top-left

    // t0: p0, p1, p2
    vec3(-0.5, -0.5,  0.5), // p0
    vec3(0.5, -0.5,  0.5), // p1
    vec3(0.5,  0.5,  0.5), // p2

    // t1: p0, p2, p3
    vec3(-0.5, -0.5,  0.5), // p0
    vec3(0.5,  0.5,  0.5),  // p2
    vec3(-0.5,  0.5,  0.5), // p3


    // -Z face (normal -Z), view from -Z looking at origin
    // p0 = (-0.5, -0.5, -0.5)  // bottom-left
    // p1 = ( 0.5, -0.5, -0.5)  // bottom-right
    // p2 = ( 0.5,  0.5, -0.5)  // top-right
    // p3 = (-0.5,  0.5, -0.5)  // top-left

    // t0: p0, p3, p2  (this ordering is what you want if you need CCW from -Z)
    vec3(-0.5, -0.5, -0.5), // p0
    vec3(-0.5,  0.5, -0.5), // p3
    vec3(0.5,  0.5, -0.5),  // p2

    // t1: p0, p2, p1
    vec3(-0.5, -0.5, -0.5), // p0
    vec3(0.5,  0.5, -0.5),  // p2
    vec3(0.5, -0.5, -0.5)   // p1
);

void main()
{
    vec3 cubePos = CUBE_VERTS[gl_VertexID % CUBE_VERTEX_COUNT];

    // Local cube vertex scaled to voxel size
    vec3 worldPos = a_position + cubePos * u_voxelSize;

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

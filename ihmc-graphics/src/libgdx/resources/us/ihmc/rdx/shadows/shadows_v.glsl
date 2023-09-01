#ifdef GL_ES
#define LOWP lowp
#define MED mediump
#define HIGH highp
precision mediump float;
#else
#define MED
#define LOWP
#define HIGH
#endif

attribute vec3 a_position;
uniform mat4 u_projViewTrans;
uniform mat4 u_worldTrans;
uniform mat4 u_lightTrans;

varying float v_positionLightTrans_x;
varying float v_positionLightTrans_y;
varying float v_positionLightTrans_z;
varying float v_positionLightTrans_w;
varying float v_position_x;
varying float v_position_y;
varying float v_position_z;
varying float v_position_w;

void precisionTransform(mat4 transform, float x, float y, float z, float w, out float xO, out float yO, out float zO, out float wO) {
    xO = transform[0].x * x + transform[1].x * y + transform[2].x * z + transform[3].x * w;
    yO = transform[0].y * x + transform[1].y * y + transform[2].y * z + transform[3].y * w;
    zO = transform[0].z * x + transform[1].z * y + transform[2].z * z + transform[3].z * w;
    wO = transform[0].w * x + transform[1].w * y + transform[2].w * z + transform[3].w * w;
}

void main()
{
    // Vertex position after transformation
    precisionTransform(u_worldTrans, a_position.x, a_position.y, a_position.z, 1.0, v_position_x, v_position_y, v_position_z, v_position_w);

    // Vertex position in the light perspective
    precisionTransform(u_lightTrans, v_position_x, v_position_y, v_position_z, v_position_w, v_positionLightTrans_x, v_positionLightTrans_y, v_positionLightTrans_z, v_positionLightTrans_w);
    gl_Position = u_projViewTrans * vec4(v_position_x, v_position_y, v_position_z, v_position_w);
}

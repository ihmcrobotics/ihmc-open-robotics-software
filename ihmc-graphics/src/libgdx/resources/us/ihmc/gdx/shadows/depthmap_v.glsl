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

varying vec4 v_position;


void main()
{
    v_position = u_worldTrans*vec4(a_position, 1.0);
    gl_Position = u_projViewTrans *v_position;
}

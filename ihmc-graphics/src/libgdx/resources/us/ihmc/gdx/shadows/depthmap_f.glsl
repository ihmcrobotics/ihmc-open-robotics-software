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

uniform float u_cameraFar;

varying vec4 v_position;
uniform vec3 u_lightPosition;

vec4 pack(HIGH float depth) {
    const vec4 bitSh = vec4(256 * 256 * 256, 256 * 256, 256, 1.0);
    const vec4 bitMsk = vec4(0, 1.0 / 256.0, 1.0 / 256.0, 1.0 / 256.0);
    vec4 comp = fract(depth * bitSh);
    comp -= comp.xxyz * bitMsk;
    return comp;
}

void main()
{
	// Simple depth calculation, just the length of the vector light-current position
    float depth = length(v_position.xyz-u_lightPosition)/u_cameraFar;
	gl_FragColor     = pack(depth);
	
}


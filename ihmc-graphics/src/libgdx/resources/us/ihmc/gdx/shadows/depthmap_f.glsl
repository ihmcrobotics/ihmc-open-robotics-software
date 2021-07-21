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

void main()
{
	// Simple depth calculation, just the length of the vector light-current position
	gl_FragColor     = vec4(length(v_position.xyz-u_lightPosition)/u_cameraFar);
	
}


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

uniform sampler2D u_depthMapDir;
uniform samplerCube u_depthMapCube;
uniform float u_cameraFar;
uniform vec3 u_lightPosition;
uniform float u_type;


varying vec4 v_position;
varying vec4 v_positionLightTrans;

float unpack (vec4 color) {
    const vec4 bitShifts = vec4(1.0 / (256.0 * 256.0 * 256.0),
    1.0 / (256.0 * 256.0),
    1.0 / 256.0,
    1);
    return dot(color , bitShifts);
}

void main()
{
	// Default is to not add any color
	float intensity=0.0; 
	// Vector light-current position
	vec3 lightDirection=v_position.xyz-u_lightPosition;
	float lenToLight=length(lightDirection)/u_cameraFar;
	// By default assume shadow
	float lenDepthMap=-1.0;
	
	// Directional light, check if in field of view and get the depth
	if(u_type==1.0){
		vec3 depth = (v_positionLightTrans.xyz / v_positionLightTrans.w)*0.5+0.5;
		if (v_positionLightTrans.z>=0.0 && (depth.x >= 0.0) && (depth.x <= 1.0) && (depth.y >= 0.0) && (depth.y <= 1.0) ) {
			lenDepthMap = unpack(texture2D(u_depthMapDir, depth.xy));
		}
	}
	// Point light, just get the depth given light vector
	else if(u_type==2.0){
		lenDepthMap = unpack(textureCube(u_depthMapCube, lightDirection));
	}
	
	// If not in shadow, add some light
	if(lenDepthMap>=lenToLight){
        intensity=0.5*(1.0-lenToLight);
    }
	
	gl_FragColor     = vec4(intensity);

}


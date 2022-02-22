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
uniform float u_lightPosition_x;
uniform float u_lightPosition_y;
uniform float u_lightPosition_z;
uniform float u_type;


varying float v_positionLightTrans_x;
varying float v_positionLightTrans_y;
varying float v_positionLightTrans_z;
varying float v_positionLightTrans_w;
varying float v_position_x;
varying float v_position_y;
varying float v_position_z;
varying float v_position_w;

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
	float intensity = 0.0;

	// Vector light-current position
    float lightDirection_x = v_position_x - u_lightPosition_x;
    float lightDirection_y = v_position_y - u_lightPosition_y;
    float lightDirection_z = v_position_z - u_lightPosition_z;

	float lenToLight = sqrt((lightDirection_x * lightDirection_x) + (lightDirection_y * lightDirection_y) + (lightDirection_z * lightDirection_z)) / u_cameraFar;
	// By default assume shadow
	float lenDepthMap = -1.0;
	
	// Directional light, check if in field of view and get the depth
	if (u_type == 1.0){
        float depth_x = v_positionLightTrans_x / v_positionLightTrans_w * 0.5 + 0.5;
        float depth_y = v_positionLightTrans_y / v_positionLightTrans_w * 0.5 + 0.5;
        float depth_z = v_positionLightTrans_z / v_positionLightTrans_w * 0.5 + 0.5;

		if (v_positionLightTrans_z >= 0.0 && (depth_x >= 0.0) && (depth_y <= 1.0) && (depth_y >= 0.0) && (depth_y <= 1.0) ) {
			lenDepthMap = unpack(texture2D(u_depthMapDir, vec2(depth_x, depth_y)));
		}
	}
	// Point light, just get the depth given light vector
	else if (u_type == 2.0){
		lenDepthMap = unpack(textureCube(u_depthMapCube, vec3(lightDirection_x, lightDirection_y, lightDirection_z)));
	}
	
	// If not in shadow, add some light
	if (lenDepthMap > lenToLight * 1.0028) { //we increase lenToLight very slightly here to counter inaccuracies that come with distance. These numbers are very precise, so a slight change causes a very large effect
        intensity = 0.5 * (1.0 - lenToLight);
    }
	
	gl_FragColor = vec4(intensity);
}


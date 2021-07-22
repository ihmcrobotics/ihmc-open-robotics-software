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

uniform sampler2D u_diffuseTexture;
uniform samplerCube u_depthMap;
uniform float u_cameraFar;
uniform vec3 u_lightPosition;


varying vec2 v_texCoords0;
varying float v_intensity;
varying vec4 v_positionLightTrans;
varying vec4 v_position;

void main()
{
    vec4 finalColor  = texture2D(u_diffuseTexture, v_texCoords0);
    finalColor.rgb   = finalColor.rgb*v_intensity;

    // Make sure the point is in the field of view of the light
    // and also that it is not behind it
    vec3 lightDirection=v_position.xyz-u_lightPosition;
    float lenToLight=length(lightDirection)/u_cameraFar;
    float lenDepthMap= textureCube(u_depthMap, lightDirection).a;
    // If can not be viewed by light > shadows
    if(lenDepthMap<lenToLight-0.0025){
        finalColor.rgb*=0.4;
    }else{
        finalColor.rgb*=0.4+0.6*(1.0-lenToLight);
    }

    gl_FragColor     = finalColor;
}


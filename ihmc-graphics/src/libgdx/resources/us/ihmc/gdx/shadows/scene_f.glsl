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
uniform sampler2D u_shadows;
uniform float u_screenWidth;
uniform float u_screenHeight;

varying vec2 v_texCoords0;
varying float v_intensity;

void main()
{
	vec4 finalColor  = texture2D(u_diffuseTexture, v_texCoords0);
	finalColor.rgb   = finalColor.rgb*v_intensity;

	// Retrieve the shadow color from shadow map
	vec2 c= gl_FragCoord.xy;
	c.x/=u_screenWidth;
	c.y/=u_screenHeight;
	vec4 color=texture2D(u_shadows,c);
	
	// Apply shadow
	finalColor.rgb*=(0.4+0.6*color.a);
	
	gl_FragColor     = finalColor;
	
	
}


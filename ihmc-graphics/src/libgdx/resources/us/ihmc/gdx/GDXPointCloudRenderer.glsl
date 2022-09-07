#type vertex
#version 410

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color;
layout(location = 2) in float a_size;

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;
uniform sampler2D u_diffuseTexture;
uniform int u_multiColor;

void main()
{
//	float halfSize = 0.5 * a_size;
//	vec4 pointInCameraFrame = u_viewTrans * vec4(a_position, 1);
//	vec4 cornerPositionInScreen = u_projTrans * vec4(halfSize, halfSize, pointInCameraFrame.z, pointInCameraFrame.w);
////	gl_PointSize = u_screenWidth * cornerPositionInScreen.x / cornerPositionInScreen.w;
////	gl_PointSize = u_screenWidth / 100.0;
//	gl_PointSize = cornerPositionInScreen.x;
////	gl_PointSize = cornerPositionInScreen.x / cornerPositionInScreen.w;

	//	vec2 projectedSize = u_screenWidth * projectedSpriteCorner.xy / projectedSpriteCorner.w;
	//	gl_PointSize = 0.25 * (projectedSize.x + projectedSize.y);

	vec4 pointInCameraFrame = u_viewTrans * vec4(a_position, 1);
	vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);
	// In VR, this value should be 0 but isn't for either the right or left eyes. Smoking gun. Not sure yet. TODO: Fix
	float shouldBeZeroButIsntSometimes = u_screenWidth * projectedSpriteCornerZero.x / projectedSpriteCornerZero.w;

	vec4 projectedSpriteCorner = u_projTrans * vec4(a_size, a_size, pointInCameraFrame.z, pointInCameraFrame.w);
	float projectedSize = u_screenWidth * projectedSpriteCorner.x / projectedSpriteCorner.w;
	if (shouldBeZeroButIsntSometimes >= 0.0)
	{
		gl_PointSize = 0.5 * (projectedSize - shouldBeZeroButIsntSometimes);
	}
    else // in VR right eye
	{
		gl_PointSize = 0.5 * abs(shouldBeZeroButIsntSometimes - projectedSize);
	}

	gl_Position = u_projTrans * pointInCameraFrame;
//	gl_Position = u_projTrans * vec4(halfSize, halfSize, pointInCameraFrame.z, pointInCameraFrame.w);;

	v_color = a_color;
}

#type fragment
#version 410

in vec4 v_color;

out vec4 color;
out float out_processedDepth;

void main()
{
	color = v_color;

	// This is so the points can be detected by the depth sensors
	out_processedDepth = 2.0 * gl_FragCoord.z - 1.0; // Normalized to -1.0 to 1.0
}

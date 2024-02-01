#type vertex
#version 410

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color;
layout(location = 2) in float a_size;

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;
uniform int u_multiColor;

void main()
{
	vec4 pointInCameraFrame = u_viewTrans * vec4(a_position.x, a_position.y, a_position.z, 1);
	vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);

	vec4 projectedSpriteCorner = u_projTrans * vec4(a_size, a_size, pointInCameraFrame.z, pointInCameraFrame.w);
	float projectedSize = u_screenWidth * projectedSpriteCorner.x / projectedSpriteCorner.w;

	gl_PointSize = 0.5 * (projectedSize);

	gl_Position = u_projTrans * pointInCameraFrame;

	v_color = a_color;
}

#type fragment
#version 410

in vec4 v_color;
out vec4 color;

void main()
{
	color = v_color;
}

#type vertex
#version 330

in vec3 a_position;
in vec4 a_color;
in vec3 a_sizeAndRotation;

out vec4 diffuseColor;

void main()
{
	diffuseColor = a_color;
//	diffuseColor = vec4(1, 0, 0, 1);
}

#type fragment
#version 330

in vec3 a_position;
in vec4 a_color;
in vec3 a_sizeAndRotation;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;
uniform sampler2D u_diffuseTexture;

void main()
{
	float halfSize = 0.5 * a_sizeAndRotation.x;
	vec4 eyePos = u_viewTrans * vec4(a_position, 1);
	vec4 projCorner = u_projTrans * vec4(halfSize, halfSize, eyePos.z, eyePos.w);
	gl_PointSize = u_screenWidth * projCorner.x / projCorner.w;
	gl_Position = u_projTrans * eyePos;
}

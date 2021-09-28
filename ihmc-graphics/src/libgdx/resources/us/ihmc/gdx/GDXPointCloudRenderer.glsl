#type vertex
#version 330

in vec3 a_position;
in vec4 a_color;
in vec3 a_sizeAndRotation;

out vec4 v_Color;

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

	v_Color = a_color;
}

#type fragment
#version 330

in vec4 v_Color;

out vec4 color;

void main()
{
	// color = v_color; // Try this if solid color works.
	color = vec4(0, 1, 0, 1);
}
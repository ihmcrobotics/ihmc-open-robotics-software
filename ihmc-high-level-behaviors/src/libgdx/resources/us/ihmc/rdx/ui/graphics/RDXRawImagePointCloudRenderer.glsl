#type vertex
#version 410

#define M_PI_F 3.1415927f

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color;
layout(location = 2) in float a_size;

out vec4 v_color;

// Generally needed uniforms
uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;

// Depth image uniforms
uniform vec4 u_depthIntrinsics; // fx, fy, cx, cy
uniform float u_depthDiscretization;
uniform mat4 u_depthToWorldTransform;

// #ifdef INPUT_COLOR_IMAGE
// // Color image uniforms
// uniform usampler2D u_colorTexture;
// uniform vec4 u_colorIntrinsics; // fx, fy, cx, cy
// uniform mat4 u_depthToColorTransform;
// uniform bool u_useColorImage
// #endif

// EUCLID STUFF //
vec3 transformPoint3D(vec3 point, mat4 transform)
{
	return vec3(dot(vec3(transform[0][0], transform[1][0], transform[2][0]), point) + transform[3][0],
				dot(vec3(transform[0][1], transform[1][1], transform[2][1]), point) + transform[3][1],
				dot(vec3(transform[0][2], transform[1][2], transform[2][2]), point) + transform[3][2]);
}

// COLOR STUFF //
vec4 calculateSinusoidalGradientColor(float input)
{
   	// maximum depth value
   	float m = 3.0f;
   	float a = 5.0f * input * M_PI_F / (3.0f * m) + M_PI_F / 2.0f;
   	float r = sin(a) * 192.0f + 128.0f;
   	float alpha = 255.0f;

	r = max(0.0f, min(255.0f, r));
   
   	float g = sin(a - 2.0f * M_PI_F / 3.0f) * 192.0f + 128.0f;
	g = max(0.0f, min(255.0f, g));
   
   	float b = sin(a - 4.0f * M_PI_F / 3.0f) * 192.0f + 128.0f;
	b = max(0.0f, min(255.0f, b));
   	
	return vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0);
}

float interpolate(float a, float b, float alpha)
{
	return (1.0f - alpha) * a + alpha * b;
}

vec4 calculateInterpolatedGradientColor(float input)
{
   // Using interpolation between key color points
   float r = 0.0f, g = 0.0f, b = 0.0f;
   float redR = 1.0f, redG = 0.0f, redB = 0.0f;
   float magentaR = 1.0f, magentaG = 0.0f, magentaB = 1.0f;
   float orangeR = 1.0f, orangeG = 200.0f / 255.0f, orangeB = 0.0f;
   float yellowR = 1.0f, yellowG = 1.0f, yellowB = 0.0f;
   float blueR = 0.0f, blueG = 0.0f, blueB = 1.0f;
   float greenR = 0.0f, greenG = 1.0f, greenB = 0.0f;
   float gradientSize = 0.2f;
   float gradientLength = 1.0f;
   float alpha = mod(input, gradientLength);
   if (alpha < 0.0f)
      alpha = 1.0f + alpha;
   if (alpha <= gradientSize * 1.0f)
   {
      r = interpolate(magentaR, blueR, (alpha) / gradientSize);
      g = interpolate(magentaG, blueG, (alpha) / gradientSize);
      b = interpolate(magentaB, blueB, (alpha) / gradientSize);
   }
   else if (alpha <= gradientSize * 2.0f)
   {
      r = interpolate(blueR, greenR, (alpha - gradientSize * 1.0f) / gradientSize);
      g = interpolate(blueG, greenG, (alpha - gradientSize * 1.0f) / gradientSize);
      b = interpolate(blueB, greenB, (alpha - gradientSize * 1.0f) / gradientSize);
   }
   else if (alpha <= gradientSize * 3.0f)
   {
      r = interpolate(greenR, yellowR, (alpha - gradientSize * 2.0f) / gradientSize);
      g = interpolate(greenG, yellowG, (alpha - gradientSize * 2.0f) / gradientSize);
      b = interpolate(greenB, yellowB, (alpha - gradientSize * 2.0f) / gradientSize);
   }
   else if (alpha <= gradientSize * 4.0f)
   {
      r = interpolate(yellowR, orangeR, (alpha - gradientSize * 3.0f) / gradientSize);
      g = interpolate(yellowG, orangeG, (alpha - gradientSize * 3.0f) / gradientSize);
      b = interpolate(yellowB, orangeB, (alpha - gradientSize * 3.0f) / gradientSize);
   }
   else if (alpha <= gradientSize * 5.0f)
   {
      r = interpolate(orangeR, redR, (alpha - gradientSize * 4.0f) / gradientSize);
      g = interpolate(orangeG, redG, (alpha - gradientSize * 4.0f) / gradientSize);
      b = interpolate(orangeB, redB, (alpha - gradientSize * 4.0f) / gradientSize);
   }

   return vec4(r, g, b, 1.0f);
}

void main()
{
	float depthInMeters = a_position.x * u_depthDiscretization;

	if (depthInMeters == 0.0f)
		return;

	uint x = uint(a_position.y);
	uint y = uint(a_position.z);

	vec3 depthFramePoint = vec3(depthInMeters,
								-(x - u_depthIntrinsics.z) / u_depthIntrinsics.x * depthInMeters,
								-(y - u_depthIntrinsics.w) / u_depthIntrinsics.y * depthInMeters);
	vec3 worldFramePoint = transformPoint3D(depthFramePoint, u_depthToWorldTransform);

	vec4 pointInCameraFrame = u_viewTrans * vec4(worldFramePoint, 1);
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

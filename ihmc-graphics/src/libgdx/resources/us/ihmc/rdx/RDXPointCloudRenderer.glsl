#type vertex
#version 410

#define M_PI_F 3.1415927f

// Coloring methods
#define COLOR_DEFAULT 0
#define COLOR_GRADIENT_WORLD_Z 1

/*
 * This attribute represents the (x, y, z) position 
 * of the point being rendered, in world frame. 
 */
layout(location = 0) in vec3 a_position;

// We output the color of the vertex for the fragment shader to use
out vec4 v_color;

// Needed uniforms
uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;
uniform float u_pointScale;
uniform vec4 u_defaultPointColor;
uniform int u_coloringMethod;

// COLOR STUFF //
vec4 calculateSinusoidalGradientColor(float value)
{
   // maximum depth value
   float m = 3.0f;
   float a = 5.0f * value * M_PI_F / (3.0f * m) + M_PI_F / 2.0f;

   float r = sin(a) * 192.0f + 128.0f;
   r = max(0.0f, min(255.0f, r));

   float g = sin(a - 2.0f * M_PI_F / 3.0f) * 192.0f + 128.0f;
   g = max(0.0f, min(255.0f, g));

   float b = sin(a - 4.0f * M_PI_F / 3.0f) * 192.0f + 128.0f;
   b = max(0.0f, min(255.0f, b));

   return vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0);
}

void main()
{
   vec4 pointColor;

   if (u_coloringMethod == COLOR_GRADIENT_WORLD_Z)
      pointColor = calculateSinusoidalGradientColor(a_position.z);
   else // u_coloringMethod == COLOR_DEFAULT, or invalid value
      pointColor = u_defaultPointColor;

   vec4 pointInCameraFrame = u_viewTrans * vec4(a_position, 1);
   vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);

   // In VR, this value should be 0 but isn't for either the right or left eyes. Smoking gun. Not sure yet. TODO: Fix
   float shouldBeZeroButIsntSometimes = u_screenWidth * projectedSpriteCornerZero.x / projectedSpriteCornerZero.w;

   vec4 projectedSpriteCorner = u_projTrans * vec4(u_pointScale, u_pointScale, pointInCameraFrame.z, pointInCameraFrame.w);
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

   v_color = pointColor;
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

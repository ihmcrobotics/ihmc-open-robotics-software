#type vertex
#version 410

/*
 * Renders pre-extracted world-frame 3D points with per-vertex RGB colour sampled from
 * the source camera image during CUDA extraction. Intended for the fused voxel map.
 */

layout(location = 0) in vec3 a_position; // world-frame (x, y, z)
layout(location = 1) in vec3 a_color;    // normalised (r, g, b) from camera image

out vec4 v_color;

uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;
uniform float u_pointScale;

void main()
{
   v_color = vec4(a_color, 1.0);

   vec4 pointInCameraFrame = u_viewTrans * vec4(a_position, 1.0);

   // Perspective-correct point-size matching the existing point cloud renderers
   vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);
   float shouldBeZero = u_screenWidth * projectedSpriteCornerZero.x / projectedSpriteCornerZero.w;

   vec4 projectedSpriteCorner = u_projTrans * vec4(u_pointScale, u_pointScale, pointInCameraFrame.z, pointInCameraFrame.w);
   float projectedSize = u_screenWidth * projectedSpriteCorner.x / projectedSpriteCorner.w;

   if (shouldBeZero >= 0.0)
      gl_PointSize = 0.5 * (projectedSize - shouldBeZero);
   else
      gl_PointSize = 0.5 * abs(shouldBeZero - projectedSize);

   gl_Position = u_projTrans * pointInCameraFrame;
}

#type fragment
#version 410

in vec4 v_color;

out vec4 color;
out float out_processedDepth;

void main()
{
   color = v_color;
   out_processedDepth = 2.0 * gl_FragCoord.z - 1.0;
}

#type vertex
#version 410

#define M_PI_F 3.1415927f

// Coloring methods
#define COLOR_DEFAULT 0
#define COLOR_GRADIENT_WORLD_Z 1
#define COLOR_GRADIENT_SENSOR_X 2
#define COLOR_FROM_IMAGE 3

/*
 * This attribute can represents the depth value
 * stored in the pixel that's being rendered as a point. 
 */
layout(location = 0) in float a_depthData;

// We output the color of the vertex for the fragment shader to use
out vec4 v_color;

// Generally needed uniforms
uniform mat4 u_viewTrans;
uniform mat4 u_projTrans;
uniform float u_screenWidth;
uniform float u_pointScale;
uniform vec4 u_defaultPointColor;
uniform int u_coloringMethod;

// Depth image uniforms
uniform vec4 u_depthIntrinsics; // fx, fy, cx, cy
uniform int u_depthImageWidth;
uniform float u_depthDiscretization;
uniform mat4 u_depthTransform;

#ifdef INPUT_COLOR_IMAGE
// Color image uniforms
uniform sampler2D u_colorTexture;
uniform vec4 u_colorIntrinsics; // fx, fy, cx, cy
uniform mat4 u_depthToColorTransform;
#endif

// EUCLID STUFF //
vec3 transformPoint3D(vec3 point, mat4 transform)
{
   return vec3(dot(vec3(transform[0][0], transform[1][0], transform[2][0]), point) + transform[3][0],
               dot(vec3(transform[0][1], transform[1][1], transform[2][1]), point) + transform[3][1],
               dot(vec3(transform[0][2], transform[1][2], transform[2][2]), point) + transform[3][2]);
}

float angle2D(vec2 a, vec2 b)
{
   float cosTheta = a.x * b.x + a.y * b.y;
   float sinTheta = a.x * b.y - a.y * b.x;
   return atan(sinTheta, cosTheta);
}

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
   vec3 worldFramePoint;
   vec4 pointColor;
   float pointSize;

   // We calculate the worldFramePoint using the depth image
   float depthInMeters = a_depthData * u_depthDiscretization;

   // No need to render if depth is zero (would be inside the sensor depth sensor)
   if (depthInMeters == 0.0f)
      return;

   uint x = gl_VertexID % u_depthImageWidth;
   uint y = gl_VertexID / u_depthImageWidth;

   vec3 depthFramePoint = vec3(depthInMeters,
                               -(x - u_depthIntrinsics.z) / u_depthIntrinsics.x * depthInMeters,
                               -(y - u_depthIntrinsics.w) / u_depthIntrinsics.y * depthInMeters);
   worldFramePoint = transformPoint3D(depthFramePoint, u_depthTransform);

   if (u_coloringMethod == COLOR_GRADIENT_WORLD_Z)
      pointColor = calculateSinusoidalGradientColor(worldFramePoint.z);
   else if (u_coloringMethod == COLOR_GRADIENT_SENSOR_X)
      pointColor = calculateSinusoidalGradientColor(depthInMeters);
#ifdef INPUT_COLOR_IMAGE
   else if (u_coloringMethod == COLOR_FROM_IMAGE)
   {
      vec3 colorFramePoint = transformPoint3D(depthFramePoint, u_depthToColorTransform);

      float yaw = -angle2D(vec2(1.0f, 0.0f), colorFramePoint.xy);
      float pitch = -angle2D(vec2(1.0f, 0.0f), colorFramePoint.xz);

      float pixelX = round(u_colorIntrinsics.z + u_colorIntrinsics.x * tan(yaw));
      float pixelY = round(u_colorIntrinsics.w + u_colorIntrinsics.y * tan(pitch));
            
      ivec2 textureSize = textureSize(u_colorTexture, 0);
      bool pixelInBounds = pixelX >= 0.0f && pixelX < textureSize.x && pixelY >= 0.0f && pixelY < textureSize.y;
      if (pixelInBounds)
      {
         vec4 colorImagePixel = texture(u_colorTexture, vec2(pixelX / textureSize.x, pixelY / textureSize.y));
         pointColor = colorImagePixel;
      }
      else
         pointColor = calculateSinusoidalGradientColor(depthInMeters);
   }
#endif
   else
      pointColor = u_defaultPointColor;

   pointSize = depthInMeters * u_pointScale;

   vec4 pointInCameraFrame = u_viewTrans * vec4(worldFramePoint, 1);
   vec4 projectedSpriteCornerZero = u_projTrans * vec4(0.0, 0.0, pointInCameraFrame.z, pointInCameraFrame.w);

   // In VR, this value should be 0 but isn't for either the right or left eyes. Smoking gun. Not sure yet. TODO: Fix
   float shouldBeZeroButIsntSometimes = u_screenWidth * projectedSpriteCornerZero.x / projectedSpriteCornerZero.w;

   vec4 projectedSpriteCorner = u_projTrans * vec4(pointSize, pointSize, pointInCameraFrame.z, pointInCameraFrame.w);
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

#define GRADIENT_MODE_WORLD_Z 0
#define GRADIENT_MODE_SENSOR_X 1

kernel void createPointCloud(read_only image2d_t depthImageDiscretized,
                             read_only image2d_t colorRGBAImage,
                             global float* finalPointFloatBuffer,
                             global float* parameters,
                             global float* depthToWorldTransform,
                             global float* depthToColorTransform)
{
   float colorFocalLengthXPixels = parameters[0];
   float colorFocalLengthYPixels = parameters[1];
   float colorPrincipalPointXPixels = parameters[2];
   float colorPrincipalPointYPixels = parameters[3];
   float focalLengthPixelsX = parameters[4];
   float focalLengthPixelsY = parameters[5];
   float dephtPrincipalPointXPixels = parameters[6];
   float dephtPrincipalPointYPixels = parameters[7];

   int depthImageWidth = parameters[7];
   int depthImageHeight = parameters[8];
   int colorImageWidth = parameters[9];
   int colorImageHeight = parameters[10];
   float discreteResolution = parameters[11];
   bool useSensorColor = parameters[12];
   int gradientMode = parameters[13];
   bool sinusoidal = parameters[14];

   int x = get_global_id(0);
   int y = get_global_id(1);

   float eyeDepthInMeters = read_imageui(depthImageDiscretized, (int2) (x, y)).x * discreteResolution;

   float3 depthFramePoint = (float3) (eyeDepthInMeters,
                            -(x - dephtPrincipalPointXPixels) / focalLengthPixelsX * eyeDepthInMeters,
                            -(y - dephtPrincipalPointYPixels) / focalLengthPixelsY * eyeDepthInMeters);

   float3 worldFramePoint = transformPoint3D32(depthFramePoint, depthToWorldTransform);
   float3 colorFramePoint = transformPoint3D32(depthFramePoint, depthToColorTransform);

   float cmosToPixelsX = colorImageWidth / cmosWidth;
   float cmosToPixelsY = colorImageHeight / cmosHeight;

   // Flip because positive yaw is to the left, but image coordinates go to the right
   float yaw = -angle(1.0f, 0.0f, colorFramePoint.x, colorFramePoint.y);
   float distanceFromSensorCenterX = focalLength * tan(yaw);
   float distanceFromSensorLeftX = distanceFromSensorCenterX + halfCMOSWidth;
   int pixelIndexX = (int) round(distanceFromSensorLeftX * cmosToPixelsX);
   bool pixelInBounds = intervalContains(pixelIndexX, 0, colorImageWidth);

   float pitch = -angle(1.0f, 0.0f, colorFramePoint.x, colorFramePoint.z);
   float distanceFromSensorCenterY = focalLength * tan(pitch);
   float distanceFromSensorTopX = distanceFromSensorCenterY + halfCMOSHeight;
   int pixelIndexY = (int) round(distanceFromSensorTopX * cmosToPixelsY);
   pixelInBounds &= intervalContains(pixelIndexY, 0, colorImageHeight);

   int color;
   if (useSensorColor && pixelInBounds)
   {
      uint4 rgba8888Color = read_imageui(colorRGBAImage, (int2) (pixelIndexX, pixelIndexY));
      color = (rgba8888Color.x << 24) | (rgba8888Color.y << 16) | (rgba8888Color.z << 8) | 255;
   }
   else if (gradientMode == GRADIENT_MODE_WORLD_Z)
   {
      color = calculateGradientColor((float) worldFramePoint.z, sinusoidal);
   }
   else // GRADIENT_MODE_SENSOR_X
   {
      color = calculateGradientColor((float) eyeDepthInMeters, sinusoidal);
   }

   int pointStartIndex = (depthImageWidth * y + x) * 8;

   finalPointFloatBuffer[pointStartIndex]     = worldFramePoint.x;
   finalPointFloatBuffer[pointStartIndex + 1] = worldFramePoint.y;
   finalPointFloatBuffer[pointStartIndex + 2] = worldFramePoint.z;
   finalPointFloatBuffer[pointStartIndex + 3] = color;

   int rInt = (color >> 24) & 0xFF;
   int gInt = (color >> 16) & 0xFF;
   int bInt = (color >> 8) & 0xFF;
   int aInt = color & 0xFF;

   float r = rInt / 255.0f;
   float g = gInt / 255.0f;
   float b = bInt / 255.0f;
   float a = aInt / 255.0f;

   finalPointFloatBuffer[pointStartIndex + 3] = r;
   finalPointFloatBuffer[pointStartIndex + 4] = g;
   finalPointFloatBuffer[pointStartIndex + 5] = b;
   finalPointFloatBuffer[pointStartIndex + 6] = a;

   finalPointFloatBuffer[pointStartIndex + 7] = 0.01f;
}
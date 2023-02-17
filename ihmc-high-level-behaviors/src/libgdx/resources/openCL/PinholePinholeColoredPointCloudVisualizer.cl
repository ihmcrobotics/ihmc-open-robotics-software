kernel void createPointCloud(read_only image2d_t depthImageMeters, 
                             read_only image2d_t colorRGBAImage,
                             global float* finalPointFloatBuffer,
                             global float* parameters,
                             global int sinusoidalPatternEnabled,
                             global float* sensorTransformToWorld)
{
   // for 3 modes of coloring
   enum VIEWMODE
   {
      COLOR = 1,
      DEPTH = 2,
      HEIGHT = 3
   };
   enum VIEWMODE viewMode;
   viewMode = COLOR;

   float focalLength = parameters[0];
   float cmosWidth = parameters[1];
   float cmosHeight = parameters[2];
   float halfCMOSWidth = cmosWidth / 2.0f;
   float halfCMOSHeight = cmosHeight / 2.0f;

   float principalOffsetXPixels = parameters[3];
   float principalOffsetYPixels = parameters[4];
   float focalLengthPixelsX = parameters[5];
   float focalLengthPixelsY = parameters[6];
   int depthImageWidth = parameters[7];
   int depthImageHeight = parameters[8];
   int colorImageWidth = parameters[9];
   int colorImageHeight = parameters[10];

   bool sinusoidal = sinusoidalPatternEnabled;

   int x = get_global_id(0);
   int y = get_global_id(1);

   float eyeDepthInMeters = read_imagef(depthImageMeters, (int2) (x, y)).x;

   float zUp3DX = eyeDepthInMeters;
   float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixelsX * eyeDepthInMeters;
   float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixelsY * eyeDepthInMeters;

   float cmosToPixelsX = colorImageWidth / cmosWidth;
   float cmosToPixelsY = colorImageHeight / cmosHeight;

   // Flip because positive yaw is to the left, but image coordinates go to the right
   float yaw = -angle(1.0f, 0.0f, zUp3DX, zUp3DY);
   float distanceFromSensorCenterX = focalLength * tan(yaw);
   float distanceFromSensorLeftX = distanceFromSensorCenterX + halfCMOSWidth;
   int pixelIndexX = (int) round(distanceFromSensorLeftX * cmosToPixelsX);
   bool pixelInBounds = intervalContains(pixelIndexX, 0, colorImageWidth);

   float pitch = -angle(1.0f, 0.0f, zUp3DX, zUp3DZ);
   float distanceFromSensorCenterY = focalLength * tan(pitch);
   float distanceFromSensorTopX = distanceFromSensorCenterY + halfCMOSHeight;
   int pixelIndexY = (int) round(distanceFromSensorTopX * cmosToPixelsY);
   pixelInBounds &= intervalContains(pixelIndexY, 0, colorImageHeight);

   float3 worldFramePoint = transform(zUp3DX, zUp3DY, zUp3DZ, sensorTransformToWorld);

   int color;
   if (viewMode == COLOR)
   {
      if (pixelInBounds)
      {
         uint4 rgba8888Color = read_imageui(colorRGBAImage, (int2) (pixelIndexX, pixelIndexY));
         color = (rgba8888Color.x << 24) | (rgba8888Color.y << 16) | (rgba8888Color.z << 8) | 255;
      }
      else
      {
         color = calculateGradientColor((float) worldFramePoint.z, sinusoidal);
      }
   }
   else if (viewMode == DEPTH)
   {
      color = calculateGradientColor((float) eyeDepthInMeters, sinusoidal);
   }
   else
   {
      color = calculateGradientColor((float) worldFramePoint.z, sinusoidal);
   }

   int pointStartIndex = (depthImageWidth * y + x) * 8;

   finalPointFloatBuffer[pointStartIndex] = worldFramePoint.x;
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
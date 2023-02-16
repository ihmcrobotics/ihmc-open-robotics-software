// TODO: float vs double in here? pick one, probably double for all the parameters
kernel void
createPointCloud(read_only image2d_t depthImageMeters, read_only image2d_t colorRGBAImage, global float *finalPointFloatBuffer, global float *parameters)
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
   float halfCMOSWidth = cmosWidth / 2.0;
   float halfCMOSHeight = cmosHeight / 2.0;

   float translationX = parameters[3];
   float translationY = parameters[4];
   float translationZ = parameters[5];
   float rotationMatrixM00 = parameters[6];
   float rotationMatrixM01 = parameters[7];
   float rotationMatrixM02 = parameters[8];
   float rotationMatrixM10 = parameters[9];
   float rotationMatrixM11 = parameters[10];
   float rotationMatrixM12 = parameters[11];
   float rotationMatrixM20 = parameters[12];
   float rotationMatrixM21 = parameters[13];
   float rotationMatrixM22 = parameters[14];

   float principalOffsetXPixels = parameters[15];
   float principalOffsetYPixels = parameters[16];
   float focalLengthPixelsX = parameters[17];
   float focalLengthPixelsY = parameters[18];
   int depthImageWidth = parameters[19];
   int depthImageHeight = parameters[20];
   int colorImageWidth = parameters[21];
   int colorImageHeight = parameters[22];

   bool sinusoidal = parameters[23] > 0.5; // TODO: Create a boolean OpenCL buffer for these binary parameters

   int x = get_global_id(0);
   int y = get_global_id(1);

   // printf("OpenCLKernel -> (x: %d, y: %d)\n", x, y);

   float eyeDepthInMeters = read_imagef(depthImageMeters, (int2) (x, y)).x;

   float zUp3DX = eyeDepthInMeters;
   float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixelsX * eyeDepthInMeters;
   float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixelsY * eyeDepthInMeters;

   float cmosToPixelsX = colorImageWidth / cmosWidth;
   float cmosToPixelsY = colorImageHeight / cmosHeight;

   // Flip because positive yaw is to the left, but image coordinates go to the right
   float yaw = -angle(1.0, 0.0, zUp3DX, zUp3DY);
   double distanceFromSensorCenterX = focalLength * tan(yaw);
   double distanceFromSensorLeftX = distanceFromSensorCenterX + halfCMOSWidth;
   int pixelIndexX = (int) round(distanceFromSensorLeftX * cmosToPixelsX);
   bool pixelInBounds = intervalContains(pixelIndexX, 0, colorImageWidth);

   double pitch = -angle(1.0, 0.0, zUp3DX, zUp3DZ);
   double distanceFromSensorCenterY = focalLength * tan(pitch);
   double distanceFromSensorTopX = distanceFromSensorCenterY + halfCMOSHeight;
   int pixelIndexY = (int) round(distanceFromSensorTopX * cmosToPixelsY);
   pixelInBounds &= intervalContains(pixelIndexY, 0, colorImageHeight);

   float4 worldFramePoint = transform(zUp3DX, zUp3DY, zUp3DZ, translationX, translationY, translationZ, rotationMatrixM00, rotationMatrixM01, rotationMatrixM02,
                                      rotationMatrixM10, rotationMatrixM11, rotationMatrixM12, rotationMatrixM20, rotationMatrixM21, rotationMatrixM22);

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
         color = calculateGradientColor((double) worldFramePoint.z, sinusoidal);
         //          color = (255 << 24) | (255 << 16) | (255 << 8) | 255; // white is default
      }
   }
   else if (viewMode == DEPTH)
   {
      //              float4 rgb = calculateGradientColorFromDepth(eyeDepthInMeters);
      //              color = ((int)rgb.x << 24) | ((int)rgb.y << 16) | ((int)rgb.z << 8) | 255;
      color = calculateGradientColor((double) eyeDepthInMeters, sinusoidal);
      //              printf("%f, %f, %f\n", rgb.x,rgb.y,rgb.z);
   }
   else
   {
      //               float4 rgb = calculateGradientColorFromZDepth(zUp3DZ);
      //               color = ((int)rgb.x << 24) | ((int)rgb.y << 16) | ((int)rgb.z << 8) | 255;
      //               printf("r: %f, g: %f, b: %f\n", rgb.x,rgb.y,rgb.z);
      color = calculateGradientColor((double) worldFramePoint.z, sinusoidal);
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

   float r = rInt / 255.0;
   float g = gInt / 255.0;
   float b = bInt / 255.0;
   float a = aInt / 255.0;

   finalPointFloatBuffer[pointStartIndex + 3] = r;
   finalPointFloatBuffer[pointStartIndex + 4] = g;
   finalPointFloatBuffer[pointStartIndex + 5] = b;
   finalPointFloatBuffer[pointStartIndex + 6] = a;

   finalPointFloatBuffer[pointStartIndex + 7] = 0.01f;
}
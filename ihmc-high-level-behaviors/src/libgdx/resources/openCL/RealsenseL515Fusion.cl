kernel void createPointCloud(read_only image2d_t depthImageMeters,
                             read_only image2d_t colorRGBImage,
                             global float* pointCloudRenderingBuffer,
                             global float* parameters)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float focalLength = parameters[0];
   float cmosWidth = parameters[1];
   float cmosHeight = parameters[2];
   float halfCMOSWidth = cmosWidth / 2.0;
   float halfCMOSHeight = cmosHeight / 2.0;

   float eyeDepth = read_imagef(depthImageMeters, (int2) (x, y)).x;

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

   float zUp3DX = eyeDepth;
   float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixelsX * eyeDepth;
   float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixelsY * eyeDepth;

   float r = 1.0;
   float g = 1.0;
   float b = 1.0;
   float a = 1.0;

   float cmosToPixelsX = colorImageWidth / cmosWidth;
   float cmosToPixelsY = colorImageHeight / cmosHeight;

   // Flip because positive yaw is to the left, but image cooridinates go to the right
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

   if (pixelInBounds)
   {
      uint4 rgba8888Color = read_imageui(colorRGBImage, (int2) (pixelIndexX, pixelIndexY));
      r = rgba8888Color.x / 255.0;
      g = rgba8888Color.y / 255.0;
      b = rgba8888Color.z / 255.0;
   }

   float4 worldFramePoint = transform(zUp3DX,
                                      zUp3DY,
                                      zUp3DZ,
                                      translationX,
                                      translationY,
                                      translationZ,
                                      rotationMatrixM00,
                                      rotationMatrixM01,
                                      rotationMatrixM02,
                                      rotationMatrixM10,
                                      rotationMatrixM11,
                                      rotationMatrixM12,
                                      rotationMatrixM20,
                                      rotationMatrixM21,
                                      rotationMatrixM22);

   int pointStartIndex = (depthImageWidth * y + x) * 8;
   pointCloudRenderingBuffer[pointStartIndex]     = worldFramePoint.x;
   pointCloudRenderingBuffer[pointStartIndex + 1] = worldFramePoint.y;
   pointCloudRenderingBuffer[pointStartIndex + 2] = worldFramePoint.z;

   pointCloudRenderingBuffer[pointStartIndex + 3] = r;
   pointCloudRenderingBuffer[pointStartIndex + 4] = g;
   pointCloudRenderingBuffer[pointStartIndex + 5] = b;
   pointCloudRenderingBuffer[pointStartIndex + 6] = a;
   pointCloudRenderingBuffer[pointStartIndex + 7] = 0.01;
}
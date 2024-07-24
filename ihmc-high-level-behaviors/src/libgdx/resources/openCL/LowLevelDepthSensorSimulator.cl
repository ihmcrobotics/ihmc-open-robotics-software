kernel void lowLevelDepthSensorSimulator(read_only image2d_t normalizedDeviceCoordinateDepthImage,
                                         read_only image2d_t noiseImage,
                                         read_only image2d_t rgba8888ColorImage,
                                         write_only image2d_t metersDepthImage,
                                         global float* pointCloudRenderingBuffer,
                                         global float* parameters)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float cameraNear = parameters[0];
   float cameraFar = parameters[1];
   float principalOffsetXPixels = parameters[2];
   float principalOffsetYPixels = parameters[3];
   float focalLengthPixels = parameters[4];
   bool calculatePointCloud = (bool) parameters[5];
   float noiseAmplitudeAtMinRange = parameters[26];
   float noiseAmplitudeAtMaxRange = parameters[27];
   float noiseAmplitudeRange = noiseAmplitudeAtMaxRange - noiseAmplitudeAtMinRange;
   bool simulateL515Noise = (bool) parameters[28];
   float randomNegativeOneToOne = read_imagef(noiseImage, (int2) (x, y)).x;
   float normalizedDeviceCoordinateZ = 2.0f * read_imagef(normalizedDeviceCoordinateDepthImage, (int2) (x, y)).x - 1.0f;

   bool depthIsZero = normalizedDeviceCoordinateZ == 1.0f;
   float eyeDepth;
   if (depthIsZero)
   {
      eyeDepth = nan((uint) 0);
   }
   else
   {
      // From "How to render depth linearly in modern OpenGL with gl_FragCoord.z in fragment shader?"
      // https://stackoverflow.com/a/45710371/1070333
      float twoXCameraFarNear = 2.0f * cameraNear * cameraFar;
      float farPlusNear = cameraFar + cameraNear;
      float farMinusNear = cameraFar - cameraNear;
      eyeDepth = (twoXCameraFarNear / (farPlusNear - normalizedDeviceCoordinateZ * farMinusNear));

      if (simulateL515Noise)
      {
         // nominal 1.5 m read
         float rangePastAMeter = clamp(eyeDepth - 1.0, 0.0, 1.0);
         // apply the effect 50/50 at a meter or less, 100% at 2 meters or more
         if ((randomNegativeOneToOne + rangePastAMeter) > 1.0)
         {
            float imageHalfWidthFloat = parameters[6] / 2;
            float imageHalfHeightFloat = parameters[7] / 2;
            float xFromCenterFloat = x - imageHalfWidthFloat;
            if (xFromCenterFloat < 0.0)
               xFromCenterFloat = -xFromCenterFloat;
            float yFromCenterFloat = y - imageHalfHeightFloat;
            if (yFromCenterFloat < 0.0)
               yFromCenterFloat = -yFromCenterFloat;
            float maxDistance = imageHalfWidthFloat * imageHalfWidthFloat + imageHalfHeightFloat * imageHalfHeightFloat;
            float pixelDistance = xFromCenterFloat * xFromCenterFloat + yFromCenterFloat * yFromCenterFloat;
            float percentToEdge = pixelDistance / maxDistance;
            if (randomNegativeOneToOne - 0.5 + percentToEdge > 1.0)
            {
               eyeDepth = 0.0f;
            }
         }
      }

      float nearToFarInterpolation = (eyeDepth - cameraNear) / farMinusNear;
      float noiseAmplitude = noiseAmplitudeAtMinRange + (nearToFarInterpolation * noiseAmplitudeRange);
      eyeDepth += randomNegativeOneToOne * noiseAmplitude;
   }

   write_imagef(metersDepthImage, (int2) (x, y), eyeDepth);

   if (calculatePointCloud)
   {
      int imageWidth = parameters[6];
      int imageHeight = parameters[7];
      float pointSize = parameters[8];
      bool colorBasedOnWorldZ = (bool) parameters[9];
      float pointColorR = parameters[10];
      float pointColorG = parameters[11];
      float pointColorB = parameters[12];
      float pointColorA = parameters[13];
      int pointStartIndex = (imageWidth * y + x) * 8;

      if (depthIsZero)
      {
         pointCloudRenderingBuffer[pointStartIndex] = nan((uint) 0);
         pointCloudRenderingBuffer[pointStartIndex + 1] = nan((uint) 0);
         pointCloudRenderingBuffer[pointStartIndex + 2] = nan((uint) 0);
      }
      else
      {
         float translationX = parameters[14];
         float translationY = parameters[15];
         float translationZ = parameters[16];
         float rotationMatrixM00 = parameters[17];
         float rotationMatrixM01 = parameters[18];
         float rotationMatrixM02 = parameters[19];
         float rotationMatrixM10 = parameters[20];
         float rotationMatrixM11 = parameters[21];
         float rotationMatrixM12 = parameters[22];
         float rotationMatrixM20 = parameters[23];
         float rotationMatrixM21 = parameters[24];
         float rotationMatrixM22 = parameters[25];

         float zUp3DX = eyeDepth;
         float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixels * eyeDepth;
         float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixels * eyeDepth;
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

         if (pointColorR < 0.0f)
         {
            uint4 rgba8888Color = read_imageui(rgba8888ColorImage, (int2) (x, y));
            pointColorR = (rgba8888Color.x / 255.0f);
            pointColorG = (rgba8888Color.y / 255.0f);
            pointColorB = (rgba8888Color.z / 255.0f);
            pointColorA = (rgba8888Color.w / 255.0f);
         }
         else if (colorBasedOnWorldZ)
         {
            float4 rgba8888Color = calculateInterpolatedGradientColorFloat4(worldFramePoint.z);
            pointColorR = (rgba8888Color.x);
            pointColorG = (rgba8888Color.y);
            pointColorB = (rgba8888Color.z);
            pointColorA = (rgba8888Color.w);
         }

         pointCloudRenderingBuffer[pointStartIndex]     = worldFramePoint.x;
         pointCloudRenderingBuffer[pointStartIndex + 1] = worldFramePoint.y;
         pointCloudRenderingBuffer[pointStartIndex + 2] = worldFramePoint.z;
      }

      pointCloudRenderingBuffer[pointStartIndex + 3] = pointColorR;
      pointCloudRenderingBuffer[pointStartIndex + 4] = pointColorG;
      pointCloudRenderingBuffer[pointStartIndex + 5] = pointColorB;
      pointCloudRenderingBuffer[pointStartIndex + 6] = pointColorA;
      pointCloudRenderingBuffer[pointStartIndex + 7] = pointSize;
   }
}
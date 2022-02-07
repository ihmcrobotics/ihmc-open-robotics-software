float4 transform(float x,
                 float y,
                 float z,
                 float translationX,
                 float translationY,
                 float translationZ,
                 float rotationMatrixM00,
                 float rotationMatrixM01,
                 float rotationMatrixM02,
                 float rotationMatrixM10,
                 float rotationMatrixM11,
                 float rotationMatrixM12,
                 float rotationMatrixM20,
                 float rotationMatrixM21,
                 float rotationMatrixM22)
{
   float4 ret = (float4) (rotationMatrixM00 * x + rotationMatrixM01 * y + rotationMatrixM02 * z,
                          rotationMatrixM10 * x + rotationMatrixM11 * y + rotationMatrixM12 * z,
                          rotationMatrixM20 * x + rotationMatrixM21 * y + rotationMatrixM22 * z,
                          0.0f);
   ret.x += translationX;
   ret.y += translationY;
   ret.z += translationZ;
   return ret;
}

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
   float noiseAmount = read_imagef(noiseImage, (int2) (x, y)).x;
   float normalizedDeviceCoordinateZ = read_imagef(normalizedDeviceCoordinateDepthImage, (int2) (x,y)).x;

   bool depthIsZero = normalizedDeviceCoordinateZ == 0.0f;
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
      eyeDepth += noiseAmount;
   }

   write_imagef(metersDepthImage, (int2) (x, y), eyeDepth);

   if (calculatePointCloud)
   {
      int imageWidth = parameters[6];
      int imageHeight = parameters[7];
      float pointSize = parameters[8];
      float pointColorR = parameters[9];
      float pointColorG = parameters[10];
      float pointColorB = parameters[11];
      float pointColorA = parameters[12];
      int pointStartIndex = (imageWidth * y + x) * 8;

      if (depthIsZero)
      {
         pointCloudRenderingBuffer[pointStartIndex]     = nan((uint) 0);
         pointCloudRenderingBuffer[pointStartIndex + 1] = nan((uint) 0);
         pointCloudRenderingBuffer[pointStartIndex + 2] = nan((uint) 0);
      }
      else
      {
         float translationX = parameters[13];
         float translationY = parameters[14];
         float translationZ = parameters[15];
         float rotationMatrixM00 = parameters[16];
         float rotationMatrixM01 = parameters[17];
         float rotationMatrixM02 = parameters[18];
         float rotationMatrixM10 = parameters[19];
         float rotationMatrixM11 = parameters[20];
         float rotationMatrixM12 = parameters[21];
         float rotationMatrixM20 = parameters[22];
         float rotationMatrixM21 = parameters[23];
         float rotationMatrixM22 = parameters[24];
         float cameraPositionX = parameters[25];
         float cameraPositionY = parameters[26];
         float cameraPositionZ = parameters[27];

         if (pointColorR < 0.0f)
         {
            uint4 rgba8888Color = read_imageui(rgba8888ColorImage, (int2) (x, y));
            pointColorR = (rgba8888Color.w / 255.0f); // Bytes are in backwards order apparently
            pointColorG = (rgba8888Color.z / 255.0f);
            pointColorB = (rgba8888Color.y / 255.0f);
            pointColorA = (rgba8888Color.x / 255.0f);
         }

         float zUp3DX = eyeDepth;
         float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixels * eyeDepth;
         float zUp3DZ = (y - principalOffsetYPixels) / focalLengthPixels * eyeDepth;
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
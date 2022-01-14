kernel void lowLevelDepthSensorSimulator(read_only image2d_t normalizedDeviceCoordinateDepthImage,
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

   // From "How to render depth linearly in modern OpenGL with gl_FragCoord.z in fragment shader?"
   // https://stackoverflow.com/a/45710371/1070333
   float twoXCameraFarNear = 2.0f * cameraNear * cameraFar;
   float farPlusNear = cameraFar + cameraNear;
   float farMinusNear = cameraFar - cameraNear;
   float normalizedDeviceCoordinateZ = read_imagef(normalizedDeviceCoordinateDepthImage, (int2) (x,y)).x;
   float eyeDepth = (twoXCameraFarNear / (farPlusNear - normalizedDeviceCoordinateZ * farMinusNear));
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
      int pointStartIndex = (imageWidth * y + x) * 8;
      pointCloudRenderingBuffer[pointStartIndex]     = zUp3DX;
      pointCloudRenderingBuffer[pointStartIndex + 1] = zUp3DY;
      pointCloudRenderingBuffer[pointStartIndex + 2] = zUp3DZ;
      pointCloudRenderingBuffer[pointStartIndex + 3] = pointColorR;
      pointCloudRenderingBuffer[pointStartIndex + 4] = pointColorG;
      pointCloudRenderingBuffer[pointStartIndex + 5] = pointColorB;
      pointCloudRenderingBuffer[pointStartIndex + 6] = pointColorA;
      pointCloudRenderingBuffer[pointStartIndex + 7] = pointSize;
   }
}
#define GRADIENT_MODE_WORLD_Z 0
#define GRADIENT_MODE_SENSOR_X 1

kernel void createPointCloud(read_only image2d_t depthImageDiscretized,
                             read_only image2d_t colorRGBAImage,
                             global float* pointCloudVertexBuffer,
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
   float depthPrincipalPointXPixels = parameters[6];
   float depthPrincipalPointYPixels = parameters[7];
   int depthImageWidth = parameters[8];
   int depthImageHeight = parameters[9];
   int colorImageWidth = parameters[10];
   int colorImageHeight = parameters[11];
   float discreteResolution = parameters[12];
   bool useSensorColor = parameters[13];
   int gradientMode = parameters[14];
   bool sinusoidal = parameters[15];

   int x = get_global_id(0);
   int y = get_global_id(1);

   float eyeDepthInMeters = read_imageui(depthImageDiscretized, (int2) (x, y)).x * discreteResolution;

   float3 depthFramePoint = (float3) (eyeDepthInMeters,
                            -(x - depthPrincipalPointXPixels) / focalLengthPixelsX * eyeDepthInMeters,
                            -(y - depthPrincipalPointYPixels) / focalLengthPixelsY * eyeDepthInMeters);

   float3 worldFramePoint = transformPoint3D32(depthFramePoint, depthToWorldTransform);

   float4 pointColor;
   bool appliedColorFromSensor = false;
   if (useSensorColor)
   {
      // TODO: Fix this, maybe getting wrong transform in BytedecoRealsense driver
      // float3 colorFramePoint = transformPoint3D32(depthFramePoint, depthToColorTransform);
      float3 colorFramePoint = depthFramePoint;

      // Flip because positive yaw is to the left, but image coordinates go to the right
      float yaw = -angle(1.0f, 0.0f, colorFramePoint.x, colorFramePoint.y);
      int pixelCol = round(colorPrincipalPointXPixels + colorFocalLengthXPixels * tan(yaw));

      float pitch = -angle(1.0f, 0.0f, colorFramePoint.x, colorFramePoint.z);
      int pixelRow = round(colorPrincipalPointYPixels + colorFocalLengthYPixels * tan(pitch));

      bool pixelInBounds = intervalContains(pixelCol, 0, colorImageWidth) && intervalContains(pixelRow, 0, colorImageHeight);

      if (pixelInBounds)
      {
         uint4 rgba8888Color = read_imageui(colorRGBAImage, (int2) (pixelCol, pixelRow));
         // pointColor = rgba8888Color / 255.0f; ?
         pointColor.x = (rgba8888Color.x / 255.0f);
         pointColor.y = (rgba8888Color.y / 255.0f);
         pointColor.z = (rgba8888Color.z / 255.0f);
         pointColor.w = (rgba8888Color.w / 255.0f);
         appliedColorFromSensor = true;
      }
   }
   if (!appliedColorFromSensor)
   {
      if (gradientMode == GRADIENT_MODE_WORLD_Z)
      {
         pointColor = calculateGradientColorOptionFloat4(worldFramePoint.z, sinusoidal);
      }
      else // GRADIENT_MODE_SENSOR_X
      {
         pointColor = calculateGradientColorOptionFloat4(eyeDepthInMeters, sinusoidal);
      }
   }

   int pointStartIndex = (depthImageWidth * y + x) * 8;
   pointCloudVertexBuffer[pointStartIndex]     = worldFramePoint.x;
   pointCloudVertexBuffer[pointStartIndex + 1] = worldFramePoint.y;
   pointCloudVertexBuffer[pointStartIndex + 2] = worldFramePoint.z;
   pointCloudVertexBuffer[pointStartIndex + 3] = pointColor.x;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColor.y;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColor.z;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColor.w;
   pointCloudVertexBuffer[pointStartIndex + 7] = 0.01f;
}
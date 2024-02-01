#define COLOR_FOCAL_LENGTH_X_PIXELS 0
#define COLOR_FOCAL_LENGTH_Y_PIXELS 1
#define COLOR_PRINCIPAL_POINT_X_PIXELS 2
#define COLOR_PRINCIPAL_POINT_Y_PIXELS 3
#define FOCAL_LENGTH_PIXELS_X 4
#define FOCAL_LENGTH_PIXELS_Y 5
#define DEPTH_PRINCIPAL_POINT_X_PIXELS 6
#define DEPTH_PRINCIPAL_POINT_Y_PIXELS 7
#define DEPTH_IMAGE_WIDTH 8
#define DEPTH_IMAGE_HEIGHT 9
#define COLOR_IMAGE_WIDTH 10
#define COLOR_IMAGE_HEIGHT 11
#define DISCRETE_RESOLUTION 12
#define USE_SENSOR_COLOR 13
#define GRADIENT_MODE 14
#define SINUSOIDAL 15
#define POINT_SIZE 16

#define GRADIENT_MODE_WORLD_Z 0
#define GRADIENT_MODE_SENSOR_X 1

kernel void computeVertexBuffer(read_only image2d_t depthImageDiscretized,
                                read_only image2d_t colorRGBAImage,
                                global float* pointCloudVertexBuffer,
                                global float* parameters,
                                global float* depthToWorldTransform,
                                global float* depthToColorTransform)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float eyeDepthInMeters = read_imageui(depthImageDiscretized, (int2) (x, y)).x * parameters[DISCRETE_RESOLUTION];

   float3 depthFramePoint = (float3) (eyeDepthInMeters,
                            -(x - parameters[DEPTH_PRINCIPAL_POINT_X_PIXELS]) / parameters[FOCAL_LENGTH_PIXELS_X] * eyeDepthInMeters,
                            -(y - parameters[DEPTH_PRINCIPAL_POINT_Y_PIXELS]) / parameters[FOCAL_LENGTH_PIXELS_Y] * eyeDepthInMeters);

   float3 worldFramePoint = transformPoint3D32(depthFramePoint, depthToWorldTransform);

   float4 pointColor;
   bool appliedColorFromSensor = false;
   if (parameters[USE_SENSOR_COLOR])
   {
      float3 colorFramePoint = transformPoint3D32(depthFramePoint, depthToColorTransform);

      // Flip because positive yaw is to the left, but image coordinates go to the right
      float yaw = -angle(1.0f, 0.0f, colorFramePoint.x, colorFramePoint.y);
      int pixelCol = round(parameters[COLOR_PRINCIPAL_POINT_X_PIXELS] + parameters[COLOR_FOCAL_LENGTH_X_PIXELS] * tan(yaw));

      float pitch = -angle(1.0f, 0.0f, colorFramePoint.x, colorFramePoint.z);
      int pixelRow = round(parameters[COLOR_PRINCIPAL_POINT_Y_PIXELS] + parameters[COLOR_FOCAL_LENGTH_Y_PIXELS] * tan(pitch));

      bool pixelInBounds = intervalContains(pixelCol, 0, parameters[COLOR_IMAGE_WIDTH]) && intervalContains(pixelRow, 0, parameters[COLOR_IMAGE_HEIGHT]);

      if (pixelInBounds)
      {
         uint4 rgba8888Color = read_imageui(colorRGBAImage, (int2) (pixelCol, pixelRow));
         pointColor = convert_float4(rgba8888Color) / 255.0f;
         appliedColorFromSensor = true;
      }
   }
   if (!appliedColorFromSensor)
   {
      if (parameters[GRADIENT_MODE] == GRADIENT_MODE_WORLD_Z)
      {
         pointColor = calculateGradientColorOptionFloat4(worldFramePoint.z, parameters[SINUSOIDAL]);
      }
      else // GRADIENT_MODE_SENSOR_X
      {
         pointColor = calculateGradientColorOptionFloat4(eyeDepthInMeters, parameters[SINUSOIDAL]);
      }
   }

   int pointStartIndex = (parameters[DEPTH_IMAGE_WIDTH] * y + x) * 8;
   pointCloudVertexBuffer[pointStartIndex]     = worldFramePoint.x;
   pointCloudVertexBuffer[pointStartIndex + 1] = worldFramePoint.y;
   pointCloudVertexBuffer[pointStartIndex + 2] = worldFramePoint.z;
   pointCloudVertexBuffer[pointStartIndex + 3] = pointColor.x;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColor.y;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColor.z;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColor.w;
   pointCloudVertexBuffer[pointStartIndex + 7] = parameters[POINT_SIZE] * eyeDepthInMeters;
}
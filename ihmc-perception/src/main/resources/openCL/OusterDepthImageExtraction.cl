#define COLUMNS_PER_FRAME 0
#define MEASUREMENT_BLOCK_SIZE 1
#define HEADER_BLOCK_BYTES 2
#define CHANNEL_DATA_BLOCK_BYTES 3

kernel void extractDepthImage(global float* parameters,
                              global unsigned char* lidarFrameBuffer,
                              read_write image2d_t depthImage16UC1)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   int bytesToColumnDataBlockStart = x * parameters[MEASUREMENT_BLOCK_SIZE] + parameters[HEADER_BLOCK_BYTES] + y * parameters[CHANNEL_DATA_BLOCK_BYTES];

   // Ouster data is little endian
   unsigned char range_MSB = lidarFrameBuffer[bytesToColumnDataBlockStart];
   unsigned char range_LSB = lidarFrameBuffer[bytesToColumnDataBlockStart + 1];
   // OpenCV is little endian
   unsigned short range = (range_LSB << 8) | range_MSB;

   write_imageui(depthImage16UC1, (int2) (x, y), (uint4) (range, 0, 0, 0));
}

#define DEPTH_IMAGE_WIDTH 0
#define DEPTH_IMAGE_HEIGHT 1
#define LIDAR_ORIGIN_TO_BEAM_ORIGIN 2
#define DISCRETE_RESOLUTION 3

kernel void computePointCloud(global float* parameters,
                              global float* altitudeAngles,
                              global float* azimuthAngles,
                              global float* ousterToWorldTransform,
                              read_only image2d_t discretizedDepthImage,
                              global float* pointCloudBuffer)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float eyeDepthInMeters = read_imageui(discretizedDepthImage, (int2) (x, y)).x * parameters[DISCRETE_RESOLUTION];

   float encoderAngle = 2.0f * M_PI_F * (1.0f - ((float) x / (float) parameters[DEPTH_IMAGE_WIDTH]));
   float azimuthAngle = -azimuthAngles[y];
   float altitudeAngle = altitudeAngles[y];

   // This uses the model from the user manual
   float beamLength = eyeDepthInMeters - parameters[LIDAR_ORIGIN_TO_BEAM_ORIGIN]; // Subtract the length of the sensor arm
   float3 ousterFramePoint = (float3)
      (-beamLength * cos(encoderAngle + azimuthAngle) * cos(altitudeAngle) + parameters[LIDAR_ORIGIN_TO_BEAM_ORIGIN] * cos(encoderAngle),
       -beamLength * sin(encoderAngle + azimuthAngle) * cos(altitudeAngle) + parameters[LIDAR_ORIGIN_TO_BEAM_ORIGIN] * sin(encoderAngle),
       beamLength * sin(altitudeAngle));

   float3 worldFramePoint = transformPoint3D32(ousterFramePoint, ousterToWorldTransform);

   int pointStartIndex = (parameters[DEPTH_IMAGE_WIDTH] * y + x) * 3;

   if (eyeDepthInMeters == 0.0f)
   {
      pointCloudBuffer[pointStartIndex]     = nan((uint) 0);
      pointCloudBuffer[pointStartIndex + 1] = nan((uint) 0);
      pointCloudBuffer[pointStartIndex + 2] = nan((uint) 0);
   }
   else
   {
      pointCloudBuffer[pointStartIndex]     = worldFramePoint.x;
      pointCloudBuffer[pointStartIndex + 1] = worldFramePoint.y;
      pointCloudBuffer[pointStartIndex + 2] = worldFramePoint.z;
   }
}
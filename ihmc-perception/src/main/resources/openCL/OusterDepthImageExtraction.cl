#define COLUMNS_PER_FRAME 0
#define MEASUREMENT_BLOCK_SIZE 1
#define HEADER_BLOCK_BYTES 2
#define CHANNEL_DATA_BLOCK_BYTES 3

kernel void extractDepthImage(global float* parameters,
                              global int* pixelShifts,
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

   int shiftedX = x;
   shiftedX += pixelShifts[y];

   if (shiftedX < 0)
      shiftedX += parameters[COLUMNS_PER_FRAME];
   if (shiftedX > parameters[COLUMNS_PER_FRAME] - 1)
      shiftedX -= parameters[COLUMNS_PER_FRAME];

   write_imageui(depthImage16UC1, (int2) (shiftedX, y), (uint4) (range, 0, 0, 0));
}

#define HORIZONTAL_FIELD_OF_VIEW 0
#define VERTICAL_FIELD_OF_VIEW 1
#define DEPTH_IMAGE_WIDTH 2
#define DEPTH_IMAGE_HEIGHT 3
#define DISCRETE_RESOLUTION 4
#define LIDAR_ORIGIN_TO_BEAM_ORIGIN 5

kernel void imageToPointCloud(global float* parameters,
                              read_only image2d_t discretizedDepthImage,
                              global float* pointCloudVertexBuffer)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float eyeDepthInMeters = read_imageui(discretizedDepthImage, (int2) (x, y)).x * parameters[DISCRETE_RESOLUTION];

   int xFromCenter = -x - (parameters[DEPTH_IMAGE_WIDTH] / 2); // flip
   int yFromCenter = y - (parameters[DEPTH_IMAGE_HEIGHT] / 2);

   float angleXFromCenter = xFromCenter / (float) parameters[DEPTH_IMAGE_WIDTH] * parameters[HORIZONTAL_FIELD_OF_VIEW];
   float angleYFromCenter = yFromCenter / (float) parameters[DEPTH_IMAGE_HEIGHT] * parameters[VERTICAL_FIELD_OF_VIEW];

   // Create additional rotation only transform
   float16 angledRotationMatrix = newRotationMatrix();
   angledRotationMatrix = setToPitchOrientation(angleYFromCenter, angledRotationMatrix);
   angledRotationMatrix = prependYawRotation(angleXFromCenter, angledRotationMatrix);

   float beamFramePointX = eyeDepthInMeters;
   float beamFramePointY = 0.0;
   float beamFramePointZ = 0.0;

   float4 sensorFramePoint = transform(beamFramePointX, beamFramePointY, beamFramePointZ, 0.0, 0.0, 0.0, angledRotationMatrix.s0, angledRotationMatrix.s1,
                                       angledRotationMatrix.s2, angledRotationMatrix.s3, angledRotationMatrix.s4, angledRotationMatrix.s5,
                                       angledRotationMatrix.s6, angledRotationMatrix.s7, angledRotationMatrix.s8);

   int pointStartIndex = (parameters[DEPTH_IMAGE_WIDTH] * y + x) * 3;

   if (eyeDepthInMeters == 0.0f)
   {
      pointCloudVertexBuffer[pointStartIndex] = 0.0f;
      pointCloudVertexBuffer[pointStartIndex + 1] = 0.0f;
      pointCloudVertexBuffer[pointStartIndex + 2] = 0.0f;
   }
   else
   {
      pointCloudVertexBuffer[pointStartIndex] = sensorFramePoint.x;
      pointCloudVertexBuffer[pointStartIndex + 1] = sensorFramePoint.y;
      pointCloudVertexBuffer[pointStartIndex + 2] = sensorFramePoint.z;
   }
}
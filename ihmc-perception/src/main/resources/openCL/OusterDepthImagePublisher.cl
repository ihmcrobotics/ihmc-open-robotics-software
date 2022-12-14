kernel void extractDepthImage(global float* parameters,
                              global int* pixelShifts,
                              global unsigned char* lidarFrameBuffer,
                              write_only image2d_t depthImage16UC1)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   int columnsPerFrame = parameters[0];
   int measurementBlockSize = parameters[1];
   int headerBlockBytes = parameters[2];
   int channelDataBlockBytes = parameters[3];
   int columnsPerMeasurementBlock = parameters[4];

   int shiftedX = x;
   shiftedX += pixelShifts[y];

   if (shiftedX < 0)
      shiftedX = columnsPerFrame + shiftedX;
   if (shiftedX > columnsPerFrame - 1)
      shiftedX -= columnsPerFrame;

   int bytesToColumnDataBlockStart = x * measurementBlockSize
                                     + headerBlockBytes
                                     + y * channelDataBlockBytes;

   // Ouster data is little endian
   unsigned char range_MSB = lidarFrameBuffer[bytesToColumnDataBlockStart];
   unsigned char range_LSB = lidarFrameBuffer[bytesToColumnDataBlockStart + 1];
   unsigned int range = (range_MSB << 8) | range_LSB;

   write_imageui(depthImage16UC1, (int2) (shiftedX, y), (uint4) (range, 0, 0, 0));
}
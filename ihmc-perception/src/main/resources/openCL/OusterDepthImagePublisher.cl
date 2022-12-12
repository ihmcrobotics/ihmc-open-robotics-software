kernel void extractDepthImage(global float* parameters,
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

   // For example, x is 0 - 2047; There's 16 columns per measurementBlock
   int measurementBlockIndex = x / columnsPerMeasurementBlock;

   int bytesToColumnDataBlockStart = measurementBlockIndex * measurementBlockSize
                                     + headerBlockBytes
                                     + y * channelDataBlockBytes;

   // Ouster data is little endian
   unsigned char range_MSB = lidarFrameBuffer[bytesToColumnDataBlockStart];
   unsigned char range_LSB = lidarFrameBuffer[bytesToColumnDataBlockStart + 1];
   unsigned int range = (range_MSB << 8) | range_LSB;

   write_imageui(depthImage16UC1, (int2) (x, y), (uint4) (range, 0, 0, 0));
}
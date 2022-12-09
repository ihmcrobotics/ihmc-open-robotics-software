// Header block
int HEADER_BLOCK_BITS = 128;
int HEADER_BLOCK_BYTES = HEADER_BLOCK_BITS / BITS_PER_BYTE;

// Header block contents
int TIMESTAMP_BITS = 64;
int TIMESTAMP_BYTES = TIMESTAMP_BITS / BITS_PER_BYTE;
int MEASUREMENT_ID_BITS = 16;
int MEASUREMENT_ID_BYTES = MEASUREMENT_ID_BITS / BITS_PER_BYTE;
int FRAME_ID_BITS = 16;
int FRAME_ID_BYTES = FRAME_ID_BITS / BITS_PER_BYTE;
int ENCODER_COUNT_BITS = 32;
int ENCODER_COUNT_BYTES = ENCODER_COUNT_BITS / BITS_PER_BYTE;

// Channel data block
int CHANNEL_DATA_BLOCK_BITS = 96;
int CHANNEL_DATA_BLOCK_BYTES = CHANNEL_DATA_BLOCK_BITS / BITS_PER_BYTE;

// Channel data block contents
int RANGE_MM_BITS = 20;
int RANGE_MM_BYTES = RANGE_MM_BITS / BITS_PER_BYTE;
int UNUSED_BITS = 12;
int RANGE_ROW_BYTES = (RANGE_MM_BITS + UNUSED_BITS) / BITS_PER_BYTE;
int REFLECTIVITY_BITS = 16;
int REFLECTIVITY_BYTES = REFLECTIVITY_BITS / BITS_PER_BYTE;
int SIGNAL_PHOTONS_BITS = 16;
int SIGNAL_PHOTONS_BYTES = SIGNAL_PHOTONS_BITS / BITS_PER_BYTE;
int NEAR_INFRARED_PHOTONS_BITS = 16;
int NEAR_INFRARED_PHOTONS_BYTES = NEAR_INFRARED_PHOTONS_BITS / BITS_PER_BYTE;
int MORE_UNUSED_BITS = 16;

// Measurement block status
int MEASUREMENT_BLOCK_STATUS_BITS = 32;
int MEASUREMENT_BLOCK_STATUS_BYTES = MEASUREMENT_BLOCK_STATUS_BITS / BITS_PER_BYTE;

kernel void extractDepthImage(global float* parameters,
                              unsigned char* lidarFrameBuffer,
                              image2d_t depthImage16UC1)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   int columnsPerMeasurementBlock = parameters[0]
   int pixelsPerColumn = parameters[1];
   int measurementBlockSize = parameters[2];

   int measurementBlockIndex = x / columnsPerMeasurementBlock;
   int columnIndexInMeasurementBlock = x % columnsPerMeasurementBlock;

   int bytesToColumnDataBlockStart = measurementBlockIndex * measurementBlockSize
                                     + HEADER_BLOCK_BYTES
                                     + columnIndexInMeasurementBlock * CHANNEL_DATA_BLOCK_BYTES;

   unsigned char range_LSB = lidarFrameBuffer[bytesToColumnDataBlockStart];
   unsigned char range_MSB = lidarFrameBuffer[bytesToColumnDataBlockStart + 1];
   unsigned int range = (range_MSB << 8) | range_LSB;

   write_imageui(depthImage16UC1, (int2) (x, y), (uint4) (range, 0, 0, 0));
}
kernel void extractDepthImage(global float* parameters,
                              global int* pixelShifts,
                              global unsigned char* lidarFrameBuffer,
                              read_write image2d_t depthImage16UC1)
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
   // OpenCV is little endian
   unsigned short range = (range_LSB << 8) | range_MSB;

   write_imageui(depthImage16UC1, (int2) (shiftedX, y), (uint4) (range, 0, 0, 0));
}

kernel void imageToPointCloud(global float* parameters,
                              read_only image2d_t discretizedDepthImage,
                              global float* pointCloudVertexBuffer)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float horizontalFieldOfView = parameters[0];
   float verticalFieldOfView = parameters[1];
   float translationX = parameters[2];
   float translationY = parameters[3];
   float translationZ = parameters[4];
   float rotationMatrixM00 = parameters[5];
   float rotationMatrixM01 = parameters[6];
   float rotationMatrixM02 = parameters[7];
   float rotationMatrixM10 = parameters[8];
   float rotationMatrixM11 = parameters[9];
   float rotationMatrixM12 = parameters[10];
   float rotationMatrixM20 = parameters[11];
   float rotationMatrixM21 = parameters[12];
   float rotationMatrixM22 = parameters[13];
   int depthImageWidth = parameters[14];
   int depthImageHeight = parameters[15];
   float pointResolution = parameters[16];

   float discreteResolution = 0.001f;
   float eyeDepthInMeters = read_imageui(discretizedDepthImage, (int2) (x, y)).x * discreteResolution;

   int xFromCenter = -x - (depthImageWidth / 2); // flip
   int yFromCenter = y - (depthImageHeight / 2);

   float angleXFromCenter = xFromCenter / (float) depthImageWidth * horizontalFieldOfView;
   float angleYFromCenter = yFromCenter / (float) depthImageHeight * verticalFieldOfView;

   // Create additional rotation only transform
   float16 angledRotationMatrix = newRotationMatrix();
   angledRotationMatrix = setToPitchOrientation(angleYFromCenter, angledRotationMatrix);
   angledRotationMatrix = prependYawRotation(angleXFromCenter, angledRotationMatrix);

   float beamFramePointX = eyeDepthInMeters;
   float beamFramePointY = 0.0;
   float beamFramePointZ = 0.0;

   float4 sensorFramePoint = transform(beamFramePointX,
                                       beamFramePointY,
                                       beamFramePointZ,
                                       0.0,
                                       0.0,
                                       0.0,
                                       angledRotationMatrix.s0,
                                       angledRotationMatrix.s1,
                                       angledRotationMatrix.s2,
                                       angledRotationMatrix.s3,
                                       angledRotationMatrix.s4,
                                       angledRotationMatrix.s5,
                                       angledRotationMatrix.s6,
                                       angledRotationMatrix.s7,
                                       angledRotationMatrix.s8);


   int pointStartIndex = (depthImageWidth * y + x) * 3;

   if (eyeDepthInMeters == 0.0f)
   {
      pointCloudVertexBuffer[pointStartIndex]     = 0.0f;
      pointCloudVertexBuffer[pointStartIndex + 1] = 0.0f;
      pointCloudVertexBuffer[pointStartIndex + 2] = 0.0f;
   }
   else
   {
      pointCloudVertexBuffer[pointStartIndex]     = sensorFramePoint.x;
      pointCloudVertexBuffer[pointStartIndex + 1] = sensorFramePoint.y;
      pointCloudVertexBuffer[pointStartIndex + 2] = sensorFramePoint.z;
   }
}
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

float16 prependYawRotation(float yaw, float16 matrixOriginal)
{
   double cYaw = cos(yaw);
   double sYaw = sin(yaw);
   double m00 = cYaw * matrixOriginal.s0 - sYaw * matrixOriginal.s3;
   double m01 = cYaw * matrixOriginal.s1 - sYaw * matrixOriginal.s4;
   double m02 = cYaw * matrixOriginal.s2 - sYaw * matrixOriginal.s5;
   double m10 = sYaw * matrixOriginal.s0 + cYaw * matrixOriginal.s3;
   double m11 = sYaw * matrixOriginal.s1 + cYaw * matrixOriginal.s4;
   double m12 = sYaw * matrixOriginal.s2 + cYaw * matrixOriginal.s5;
   double m20 = matrixOriginal.s6;
   double m21 = matrixOriginal.s7;
   double m22 = matrixOriginal.s8;
   matrixOriginal.s0 = m00;
   matrixOriginal.s1 = m01;
   matrixOriginal.s2 = m02;
   matrixOriginal.s3 = m10;
   matrixOriginal.s4 = m11;
   matrixOriginal.s5 = m12;
   matrixOriginal.s6 = m20;
   matrixOriginal.s7 = m21;
   matrixOriginal.s8 = m22;
   return matrixOriginal;
}

float16 setToPitchOrientation(float pitch, float16 rotationMatrix)
{
   float sinPitch = sin(pitch);
   float cosPitch = cos(pitch);
   rotationMatrix.s0 = cosPitch;
   rotationMatrix.s1 = 0.0;
   rotationMatrix.s2 = sinPitch;
   rotationMatrix.s3 = 0.0;
   rotationMatrix.s4 = 1.0;
   rotationMatrix.s5 = 0.0;
   rotationMatrix.s6 = -sinPitch;
   rotationMatrix.s7 = 0.0;
   rotationMatrix.s8 = cosPitch;
   return rotationMatrix;
}

float16 newRotationMatrix()
{
   return (float16) (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // unused values
}

int discretize(float value, float discretization)
{
    return round(value / discretization);
}

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
   // OpenCV is little endian
   unsigned short range = (range_LSB << 8) | range_MSB;

   write_imageui(depthImage16UC1, (int2) (shiftedX, y), (uint4) (range, 0, 0, 0));
}

kernel void imageToDiscretizedPointCloud(global float* parameters,
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
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

float angle(float x1, float y1, float x2, float y2)
{
   float cosTheta = x1 * x2 + y1 * y2;
   float sinTheta = x1 * y2 - y1 * x2;
   return atan2(sinTheta, cosTheta);
}

bool intervalContains(float value, float lowerEndpoint, float upperEndpoint)
{
   return value >= lowerEndpoint && value <= upperEndpoint;
}

kernel void createPointCloud(read_only image2d_t depthImageMeters,
                             global float* pointCloudRenderingBuffer,
                             global float* parameters)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float horizontalFieldOfView = parameters[0];
   float verticalFieldOfView = parameters[1];

   float eyeDepth = read_imagef(depthImageMeters, (int2) (x, y)).x;

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


   int xFromCenter = x - (depthImageWidth / 2);
   int yFromCenter = y - (depthImageHeight / 2);


   float angleXFromCenter = xFromCenter * (float) depthImageWidth / horizontalFieldOfView;
   float angleYFromCenter = yFromCenter * (float) depthImageHeight / verticalFieldOfView;

//   float zUp3DX = eyeDepth * sin(angleXFromCenter);
//   float zUp3DY = eyeDepth * sin(angleYFromCenter);
//   float zUp3DZ = eyeDepth * cos(angleXFromCenter);

   float zUp3DX = eyeDepth * cos(angleXFromCenter) * cos(angleYFromCenter);
   float zUp3DY = eyeDepth * sin(angleXFromCenter) * cos(angleYFromCenter);
   float zUp3DZ = eyeDepth * sin(angleYFromCenter);

//   float zUp3DX = eyeDepth;
//   float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixelsX * eyeDepth;
//   float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixelsY * eyeDepth;

   float r = 1.0;
   float g = 1.0;
   float b = 1.0;
   float a = 1.0;

   float4 worldFramePoint = transform(zUp3DX,
                                      zUp3DY,
                                      zUp3DZ,
                                      translationX,
                                      translationY,
                                      translationZ,
                                      rotationMatrixM00,
                                      rotationMatrixM01,
                                      rotationMatrixM02,
                                      rotationMatrixM10,
                                      rotationMatrixM11,
                                      rotationMatrixM12,
                                      rotationMatrixM20,
                                      rotationMatrixM21,
                                      rotationMatrixM22);

   int pointStartIndex = (depthImageWidth * y + x) * 8;
   pointCloudRenderingBuffer[pointStartIndex]     = worldFramePoint.x;
   pointCloudRenderingBuffer[pointStartIndex + 1] = worldFramePoint.y;
   pointCloudRenderingBuffer[pointStartIndex + 2] = worldFramePoint.z;

   pointCloudRenderingBuffer[pointStartIndex + 3] = r;
   pointCloudRenderingBuffer[pointStartIndex + 4] = g;
   pointCloudRenderingBuffer[pointStartIndex + 5] = b;
   pointCloudRenderingBuffer[pointStartIndex + 6] = a;
   pointCloudRenderingBuffer[pointStartIndex + 7] = 0.01;
}
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
   float pointSize = parameters[16];

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

   float4 worldFramePoint = transform(sensorFramePoint.x,
                                      sensorFramePoint.y,
                                      sensorFramePoint.z,
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

   if (eyeDepthInMeters == 0.0f)
   {
      pointCloudVertexBuffer[pointStartIndex]     = nan((uint) 0);
      pointCloudVertexBuffer[pointStartIndex + 1] = nan((uint) 0);
      pointCloudVertexBuffer[pointStartIndex + 2] = nan((uint) 0);
   }
   else
   {
      pointCloudVertexBuffer[pointStartIndex]     = worldFramePoint.x;
      pointCloudVertexBuffer[pointStartIndex + 1] = worldFramePoint.y;
      pointCloudVertexBuffer[pointStartIndex + 2] = worldFramePoint.z;
   }

   float4 rgba8888Color = calculateInterpolatedGradientColorFloat4(worldFramePoint.z);
   float pointColorR = (rgba8888Color.x);
   float pointColorG = (rgba8888Color.y);
   float pointColorB = (rgba8888Color.z);
   float pointColorA = (rgba8888Color.w);

   pointCloudVertexBuffer[pointStartIndex + 3] = pointColorR;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColorG;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColorB;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColorA;
   pointCloudVertexBuffer[pointStartIndex + 7] = pointSize;
}

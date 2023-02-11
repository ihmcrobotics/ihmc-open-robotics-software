kernel void imageToPointCloud(global float* parameters,
                              read_only image2d_t discretizedDepthImage,
                              read_only image2d_t fThetaFisheyeRGBA8Image,
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
   bool useFisheyeColorImage = parameters[17];
   int fisheyeImageWidth = parameters[18];
   int fisheyeImageHeight = parameters[19];
   float fisheyeFocalLengthPixelsX = parameters[20];
   float fisheyeFocalLengthPixelsY = parameters[21];
   float fisheyePrincipalPointPixelsX = parameters[22];
   float fisheyePrincipalPointPixelsY = parameters[23];
   float fisheyeTranslationX = parameters[24];
   float fisheyeTranslationY = parameters[25];
   float fisheyeTranslationZ = parameters[26];
   float fisheyeRotationMatrixM00 = parameters[27];
   float fisheyeRotationMatrixM01 = parameters[28];
   float fisheyeRotationMatrixM02 = parameters[29];
   float fisheyeRotationMatrixM10 = parameters[30];
   float fisheyeRotationMatrixM11 = parameters[31];
   float fisheyeRotationMatrixM12 = parameters[32];
   float fisheyeRotationMatrixM20 = parameters[33];
   float fisheyeRotationMatrixM21 = parameters[34];
   float fisheyeRotationMatrixM22 = parameters[35];

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


   float pointColorR;
   float pointColorG;
   float pointColorB;
   float pointColorA;
   if (useFisheyeColorImage)
   {
      float4 fisheyeFramePoint = transform(sensorFramePoint.x,
                                           sensorFramePoint.y,
                                           sensorFramePoint.z,
                                           fisheyeTranslationX,
                                           fisheyeTranslationY,
                                           fisheyeTranslationZ,
                                           fisheyeRotationMatrixM00,
                                           fisheyeRotationMatrixM01,
                                           fisheyeRotationMatrixM02,
                                           fisheyeRotationMatrixM10,
                                           fisheyeRotationMatrixM11,
                                           fisheyeRotationMatrixM12,
                                           fisheyeRotationMatrixM20,
                                           fisheyeRotationMatrixM21,
                                           fisheyeRotationMatrixM22);

      float fisheyeAngleX = -angle(1.0f, 0.0f, fisheyeFramePoint.x,  fisheyeFramePoint.y);
      float fisheyeAngleY = angle(1.0f, 0.0f, fisheyeFramePoint.x, -fisheyeFramePoint.z);

      if (fabs(fisheyeAngleX) < radians(92.5f) && fabs(fisheyeAngleY) < radians(92.5f))
      {
         int fisheyeX = fisheyePrincipalPointPixelsX + fisheyeFocalLengthPixelsX * fisheyeAngleX;
         int fisheyeY = fisheyePrincipalPointPixelsY + fisheyeFocalLengthPixelsY * fisheyeAngleY;
         if (fisheyeX >= 0 && fisheyeX < fisheyeImageWidth && fisheyeY >= 0 && fisheyeY < fisheyeImageHeight)
         {
            uint4 fisheyeColor = read_imageui(fThetaFisheyeRGBA8Image, (int2) (fisheyeX, fisheyeY));
            pointColorR = (fisheyeColor.x / 255.0f);
            pointColorG = (fisheyeColor.y / 255.0f);
            pointColorB = (fisheyeColor.z / 255.0f);
            pointColorA = (fisheyeColor.w / 255.0f);
         }
      }
      else
      {
         float4 rgba8888Color = calculateInterpolatedGradientColorFloat4(worldFramePoint.z);
         pointColorR = rgba8888Color.x;
         pointColorG = rgba8888Color.y;
         pointColorB = rgba8888Color.z;
         pointColorA = rgba8888Color.w;
      }
   }
   else
   {
      float4 rgba8888Color = calculateInterpolatedGradientColorFloat4(worldFramePoint.z);
      pointColorR = (rgba8888Color.x);
      pointColorG = (rgba8888Color.y);
      pointColorB = (rgba8888Color.z);
      pointColorA = (rgba8888Color.w);
   }

   pointCloudVertexBuffer[pointStartIndex + 3] = pointColorR;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColorG;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColorB;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColorA;
   pointCloudVertexBuffer[pointStartIndex + 7] = pointSize;
}

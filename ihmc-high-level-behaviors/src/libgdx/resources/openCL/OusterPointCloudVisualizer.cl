kernel void imageToPointCloud(global float* parameters,
                              read_only image2d_t discretizedDepthImage,
                              read_only image2d_t fThetaFisheyeRGBA8Image,
                              global float* pointCloudVertexBuffer)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float horizontalFieldOfView = parameters[0];
   float verticalFieldOfView = parameters[1];
   float ousterToWorldTranslationX = parameters[2];
   float ousterToWorldTranslationY = parameters[3];
   float ousterToWorldTranslationZ = parameters[4];
   float ousterToWorldRotationMatrixM00 = parameters[5];
   float ousterToWorldRotationMatrixM01 = parameters[6];
   float ousterToWorldRotationMatrixM02 = parameters[7];
   float ousterToWorldRotationMatrixM10 = parameters[8];
   float ousterToWorldRotationMatrixM11 = parameters[9];
   float ousterToWorldRotationMatrixM12 = parameters[10];
   float ousterToWorldRotationMatrixM20 = parameters[11];
   float ousterToWorldRotationMatrixM21 = parameters[12];
   float ousterToWorldRotationMatrixM22 = parameters[13];
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
   float fisheyeToOusterTranslationX = parameters[24];
   float fisheyeToOusterTranslationY = parameters[25];
   float fisheyeToOusterTranslationZ = parameters[26];
   float fisheyeToOusterRotationMatrixM00 = parameters[27];
   float fisheyeToOusterRotationMatrixM01 = parameters[28];
   float fisheyeToOusterRotationMatrixM02 = parameters[29];
   float fisheyeToOusterRotationMatrixM10 = parameters[30];
   float fisheyeToOusterRotationMatrixM11 = parameters[31];
   float fisheyeToOusterRotationMatrixM12 = parameters[32];
   float fisheyeToOusterRotationMatrixM20 = parameters[33];
   float fisheyeToOusterRotationMatrixM21 = parameters[34];
   float fisheyeToOusterRotationMatrixM22 = parameters[35];

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

   float4 ousterFramePoint = transform(beamFramePointX,
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

   float4 worldFramePoint = transform(ousterFramePoint.x,
                                      ousterFramePoint.y,
                                      ousterFramePoint.z,
                                      ousterToWorldTranslationX,
                                      ousterToWorldTranslationY,
                                      ousterToWorldTranslationZ,
                                      ousterToWorldRotationMatrixM00,
                                      ousterToWorldRotationMatrixM01,
                                      ousterToWorldRotationMatrixM02,
                                      ousterToWorldRotationMatrixM10,
                                      ousterToWorldRotationMatrixM11,
                                      ousterToWorldRotationMatrixM12,
                                      ousterToWorldRotationMatrixM20,
                                      ousterToWorldRotationMatrixM21,
                                      ousterToWorldRotationMatrixM22);

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
      float4 fisheyeFramePoint = transform(ousterFramePoint.x,
                                           ousterFramePoint.y,
                                           ousterFramePoint.z,
                                           fisheyeToOusterTranslationX,
                                           fisheyeToOusterTranslationY,
                                           fisheyeToOusterTranslationZ,
                                           fisheyeToOusterRotationMatrixM00,
                                           fisheyeToOusterRotationMatrixM01,
                                           fisheyeToOusterRotationMatrixM02,
                                           fisheyeToOusterRotationMatrixM10,
                                           fisheyeToOusterRotationMatrixM11,
                                           fisheyeToOusterRotationMatrixM12,
                                           fisheyeToOusterRotationMatrixM20,
                                           fisheyeToOusterRotationMatrixM21,
                                           fisheyeToOusterRotationMatrixM22);

      float angleOfIncidence = angle3D(1.0f, 0.0f, 0.0f, fisheyeFramePoint.x, fisheyeFramePoint.y, fisheyeFramePoint.z);
      if (fabs(angleOfIncidence) < radians(92.5f))
      {
         // Equidistant fisheye camera model:
         // r = f * theta
         // theta is the azimuthal angle, i.e. the angle swept on the image plane to the image point cooresponding to the fisheye frame point.
         // We use atan2(-z, -y) with negative z and y to convert from right handed Y left, Z up to image X right, Y down.
         // f is a 2D vector of focal length in pixels decomposed into X and Y so that we can tune each separately, which is useful.
         // To get the components or r, we use sin and cos of theta (i.e. the azimuthal angle).
         // r is a 2D vector of pixels from the principle point on the image to the point cooresponding to the fisheye frame point.
         // We offset r by the principle point offsets to find the pixel row and column.
         // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
         // https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function
         // https://www.ihmc.us/wp-content/uploads/2023/02/equidistant_fisheye_model-1024x957.jpeg
         float azimuthalAngle = atan2(-fisheyeFramePoint.z, -fisheyeFramePoint.y);
         int fisheyeCol = fisheyePrincipalPointPixelsX + fisheyeFocalLengthPixelsX * angleOfIncidence * cos(azimuthalAngle);
         int fisheyeRow = fisheyePrincipalPointPixelsY + fisheyeFocalLengthPixelsY * angleOfIncidence * sin(azimuthalAngle);

         if (fisheyeCol >= 0 && fisheyeCol < fisheyeImageWidth && fisheyeRow >= 0 && fisheyeRow < fisheyeImageHeight)
         {
            uint4 fisheyeColor = read_imageui(fThetaFisheyeRGBA8Image, (int2) (fisheyeCol, fisheyeRow));
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

   pointSize = pointSize * eyeDepthInMeters;

   pointCloudVertexBuffer[pointStartIndex + 3] = pointColorR;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColorG;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColorB;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColorA;
   pointCloudVertexBuffer[pointStartIndex + 7] = pointSize;
}

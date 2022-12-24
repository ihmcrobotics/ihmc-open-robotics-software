float4 createRGB(double input)
{
//   // maximum depth value
//   float blend = 192.0 / 255.0;
////   float half = 0.5;
//
//   float m = 3;
//   float PI = 3.141592;
//   float a = 5 * input * PI / (3 * m) + PI / 2;
//   float r = sin(a) * blend + 0.5;
//   float alpha = 255;
//
//   if (r < 0)
//      r = 0;
//   else if (r > 255)
//      r = 255;
//   float g = sin(a - 2 * PI / 3) * blend + 0.5;
//   if (g < 0)
//      g = 0;
//   else if (g > 255)
//      g = 255;
//   float b = sin(a - 4 * PI / 3) * blend + 0.5;
//   if (b < 0)
//      b = 255;

   // using interpolation between keu color points >>
   double r = 0, g = 0, b = 0;
   double redR = 1.0, redG = 0.0, redB = 0.0;
   double magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
   double orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
   double yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
   double blueR = 0.0, blueG = 0.0, blueB = 1.0;
   double greenR = 0.0, greenG = 1.0, greenB = 0.0;
   double gradientSize = 0.2;
   double gradientLength = 1;
   double alpha = fmod(input, gradientLength);
   if (alpha < 0)
      alpha = 1 + alpha;
   if (alpha <= gradientSize * 1)
   {
      r = interpolate(magentaR, blueR, (alpha) / gradientSize);
      g = interpolate(magentaG, blueG, (alpha) / gradientSize);
      b = interpolate(magentaB, blueB, (alpha) / gradientSize);
   }
   else if (alpha <= gradientSize * 2)
   {
      r = interpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
      g = interpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
      b = interpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
   }
   else if (alpha <= gradientSize * 3)
   {
      r = interpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
      g = interpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
      b = interpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
   }
   else if (alpha <= gradientSize * 4)
   {
      r = interpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
      g = interpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
      b = interpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
   }
   else if (alpha <= gradientSize * 5)
   {
      r = interpolate(orangeR, redR, (alpha - gradientSize * 4) / gradientSize);
      g = interpolate(orangeG, redG, (alpha - gradientSize * 4) / gradientSize);
      b = interpolate(orangeB, redB, (alpha - gradientSize * 4) / gradientSize);
   }

   return (float4) (r, g, b, 1.0);
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

//   double halfDepthImageWidth = depthImageWidth / 2.0;
//   double halfDepthImageHeight = depthImageHeight / 2.0;

   int xFromCenter = -x - (depthImageWidth / 2); // flip
   int yFromCenter = y - (depthImageHeight / 2);

//   float percentYToEdge = yFromCenter / (float) depthImageHeight;


   float angleXFromCenter = xFromCenter / (float) depthImageWidth * horizontalFieldOfView;
   float angleYFromCenter = yFromCenter / (float) depthImageHeight * verticalFieldOfView;

   // Possible simpler ways to do this, but none worked so far:

//   float zUp3DX = eyeDepth * sin(angleXFromCenter);
//   float zUp3DY = eyeDepth * sin(angleYFromCenter);
//   float zUp3DZ = eyeDepth * cos(angleXFromCenter);

//   float zUp3DX = eyeDepth * cos(angleXFromCenter) * cos(angleYFromCenter);
//   float zUp3DY = eyeDepth * sin(angleXFromCenter) * cos(angleYFromCenter);
//   float zUp3DZ = eyeDepth * sin(angleYFromCenter);

//   float zUp3DX = eyeDepth;
//   float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixelsX * eyeDepth;
//   float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixelsY * eyeDepth;

//   float4 worldFramePoint = transform(zUp3DX,
//                                      zUp3DY,
//                                      zUp3DZ,
//                                      translationX,
//                                      translationY,
//                                      translationZ,
//                                      rotationMatrixM00,
//                                      rotationMatrixM01,
//                                      rotationMatrixM02,
//                                      rotationMatrixM10,
//                                      rotationMatrixM11,
//                                      rotationMatrixM12,
//                                      rotationMatrixM20,
//                                      rotationMatrixM21,
//                                      rotationMatrixM22);

   // Create additional rotation only transform
   float16 angledRotationMatrix = newRotationMatrix();
   angledRotationMatrix = setToPitchOrientation(angleYFromCenter, angledRotationMatrix);
   angledRotationMatrix = prependYawRotation(angleXFromCenter, angledRotationMatrix);

   float beamFramePointX = eyeDepth;
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



   float r = 1.0;
   float g = 1.0;
   float b = 1.0;
   float a = 1.0;
   float4 color = createRGB(worldFramePoint.z);

   int pointStartIndex = (depthImageWidth * y + x) * 8;
   pointCloudRenderingBuffer[pointStartIndex]     = worldFramePoint.x;
   pointCloudRenderingBuffer[pointStartIndex + 1] = worldFramePoint.y;
   pointCloudRenderingBuffer[pointStartIndex + 2] = worldFramePoint.z;

   pointCloudRenderingBuffer[pointStartIndex + 3] = color.x;
   pointCloudRenderingBuffer[pointStartIndex + 4] = color.y;
   pointCloudRenderingBuffer[pointStartIndex + 5] = color.z;
   pointCloudRenderingBuffer[pointStartIndex + 6] = color.w;
   pointCloudRenderingBuffer[pointStartIndex + 7] = 0.01;
}
int createRGB(double input)
{
   // using interpolation between keu color points >>
   double r = 0,g = 0,b = 0;
   double redR     = 255.0,  redG     = 0.0,      redB     = 0.0;
   double magentaR = 255.0,  magentaG = 0.0,      magentaB = 255.0;
   double orangeR  = 255.0,  orangeG  = 200.0,    orangeB  = 0.0;
   double yellowR  = 255.0,  yellowG  = 255.0,    yellowB  = 0.0;
   double blueR    = 0.0,    blueG    = 0.0,      blueB    = 255.0;
   double greenR   = 0.0,    greenG   = 255.0,    greenB   = 0.0;

   double gradientSize = 0.2;
   double gradientLength = 1;
   double alpha = fmod(input, gradientLength);

   if(alpha < 0)
      alpha = 1 + alpha;
   if(alpha <= gradientSize * 1)
   {
      r = interpolate(magentaR, blueR, (alpha) / gradientSize);
      g = interpolate(magentaG, blueG, (alpha) / gradientSize);
      b = interpolate(magentaB, blueB, (alpha) / gradientSize);
   }
   else if(alpha <= gradientSize * 2)
   {
      r = interpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
      g = interpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
      b = interpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
   }
   else if(alpha<=gradientSize * 3)
   {
      r = interpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
      g = interpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
      b = interpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
   }
   else if(alpha<=gradientSize * 4)
   {
      r = interpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
      g = interpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
      b = interpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
   }
   else if(alpha<=gradientSize * 5)
   {
      r = interpolate(orangeR, redR, (alpha - gradientSize * 4) / gradientSize);
      g = interpolate(orangeG, redG, (alpha - gradientSize * 4) / gradientSize);
      b = interpolate(orangeB, redB, (alpha - gradientSize * 4) / gradientSize);
   }

   int color = ((int) round(r) << 24) | ((int) round(g) << 16) | ((int) round(b) << 8) | 255;
   return color;
}

kernel void discretizePoints(global float* pointCloudBuffer,
                             global int* discretizedIntBuffer,
                             global float* parameters)
{
   int n = get_global_id(0);

   int floatsPerPoint = parameters[0];
   int intsPerPoint = parameters[1];
   float discreteResolution = parameters[2];
   int segmentIndex = parameters[3];
   int pointsPerSegment = parameters[4];

   float4 worldFramePoint;
   int worldPointStartIndex = segmentIndex * pointsPerSegment * floatsPerPoint + n * floatsPerPoint;
   worldFramePoint.x = pointCloudBuffer[worldPointStartIndex];
   worldFramePoint.y = pointCloudBuffer[worldPointStartIndex + 1];
   worldFramePoint.z = pointCloudBuffer[worldPointStartIndex + 2];

   int color = createRGB((double) worldFramePoint.z);

   int discreteX = (int) round(worldFramePoint.x / discreteResolution);
   int discreteY = (int) round(worldFramePoint.y / discreteResolution);
   int discreteZ = (int) round(worldFramePoint.z / discreteResolution);

   int pointStartIndex = n * intsPerPoint;
   discretizedIntBuffer[pointStartIndex]     = discreteX;
   discretizedIntBuffer[pointStartIndex + 1] = discreteY;
   discretizedIntBuffer[pointStartIndex + 2] = discreteZ;
   discretizedIntBuffer[pointStartIndex + 3] = color;
}
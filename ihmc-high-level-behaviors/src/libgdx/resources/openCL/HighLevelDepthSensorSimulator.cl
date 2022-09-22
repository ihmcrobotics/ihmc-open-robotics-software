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

double interpolate(double a, double b, double alpha)
{
  return (1.0 - alpha) * a + alpha * b;
}

// TODO: create (R,G,B) based on eyeDepth ( distance from me )
int createRGB(double input)
{
   bool sinusoidal = false;
   if(sinusoidal)
   {
      // maximum depth value
      float m = 3;
      float PI = 3.141592;
      float a= 5*input*PI/(3*m) + PI/2;
      float r=sin(a) * 192 + 128;
      float alpha = 255;

      if(r<0) r=0;
      else if(r>255) r =255;
      float g=sin(a - 2*PI/3) * 192 + 128;
      if(g<0) g=0;
      else if(g>255) g =255;
      float b=sin(a - 4*PI/3) * 192 + 128;
      if(b<0) b=0;
      else if(b>255) b =255;
      int color = ((int)round(r) << 24) | ((int)round(g) << 16) | ((int)round(b) << 8) | 255;
      return color;
   }
   else
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
      double alpha = fmod(input,gradientLength);

      if(alpha<0) alpha = 1 + alpha;
      if(alpha <= gradientSize * 1)
      {
         r = interpolate(magentaR,blueR,(alpha) / gradientSize);
         g = interpolate(magentaG,blueG,(alpha) / gradientSize);
         b = interpolate(magentaB,blueB,(alpha) / gradientSize);
      }
      else if(alpha <= gradientSize * 2)
      {
         r = interpolate(blueR,greenR,(alpha - gradientSize * 1) / gradientSize);
         g = interpolate(blueG,greenG,(alpha - gradientSize * 1) / gradientSize);
         b = interpolate(blueB,greenB,(alpha - gradientSize * 1) / gradientSize);
      }
      else if(alpha<=gradientSize * 3)
      {
         r = interpolate(greenR,yellowR,(alpha - gradientSize * 2) / gradientSize);
         g = interpolate(greenG,yellowG,(alpha - gradientSize * 2) / gradientSize);
         b = interpolate(greenB,yellowB,(alpha - gradientSize * 2) / gradientSize);
      }
      else if(alpha<=gradientSize * 4)
      {
         r = interpolate(yellowR,orangeR,(alpha - gradientSize * 3) / gradientSize);
         g = interpolate(yellowG,orangeG,(alpha - gradientSize * 3) / gradientSize);
         b = interpolate(yellowB,orangeB,(alpha - gradientSize * 3) / gradientSize);
      }
      else if(alpha<=gradientSize * 5)
      {
         r = interpolate(orangeR,redR,(alpha - gradientSize * 4) / gradientSize);
         g = interpolate(orangeG,redG,(alpha - gradientSize * 4) / gradientSize);
         b = interpolate(orangeB,redB,(alpha - gradientSize * 4) / gradientSize);
      }
      int color = ((int)round(r) << 24) | ((int)round(g) << 16) | ((int)round(b) << 8) | 255;
      return color;
   }
}

kernel void discretizePoints(read_only image2d_t metersDepthBuffer,
                             global int* discretizedIntBuffer,
                             global float* parameters)
{
   // for 3 modes of coloring
   enum VIEWMODE{COLOR=1, DEPTH=2, HEIGHT=3};
   enum VIEWMODE viewMode;
   viewMode = COLOR;

   int x = get_global_id(0);
   int y = get_global_id(1);

   float horizontalFieldOfView = parameters[0];
   float verticalFieldOfView = parameters[1];

   float eyeDepthInMeters = read_imagef(metersDepthBuffer, (int2) (x, y)).x;

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
   float discreteResolution = parameters[16];

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

   int color;
   if(viewMode == DEPTH)
   {
      color = createRGB((double) eyeDepthInMeters);
   }
   else
   {
      color = createRGB((double) worldFramePoint.z);
   }

   int pointStartIndex = (depthImageWidth * y + x) * 4;
   int discreteX = (int) round(worldFramePoint.x / discreteResolution);
   int discreteY = (int) round(worldFramePoint.y / discreteResolution);
   int discreteZ = (int) round(worldFramePoint.z / discreteResolution);

   discretizedIntBuffer[pointStartIndex]     = discreteX;
   discretizedIntBuffer[pointStartIndex + 1] = discreteY;
   discretizedIntBuffer[pointStartIndex + 2] = discreteZ;
   discretizedIntBuffer[pointStartIndex + 3] = color;
}
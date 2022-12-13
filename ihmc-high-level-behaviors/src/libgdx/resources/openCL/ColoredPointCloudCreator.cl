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

double interpolate(double a, double b, double alpha)
{
  return (1.0 - alpha) * a + alpha * b;
}

// TODO: create (R,G,B) based on eyeDepth ( distance from me )
int createRGB(double input, bool sinusoidal)
{
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
        //    r=max(0,min(255,r));
        float g=sin(a - 2*PI/3) * 192 + 128;
        if(g<0) g=0;
        else if(g>255) g =255;
        //    g=max(0,min(255,g));
        float b=sin(a - 4*PI/3) * 192 + 128;
        if(b<0) b=0;
        else if(b>255) b =255;
        //    b=max(0,min(255,b));
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


// TODO: float vs double in here? pick one, probably double for all the parameters
kernel void createPointCloud(read_only image2d_t depthImageMeters,
                             read_only image2d_t colorRGBAImage,
                             global float* finalPointFloatBuffer,
                             global float* parameters)
{
    // for 3 modes of coloring
   enum VIEWMODE{COLOR=1, DEPTH=2, HEIGHT=3};
   enum VIEWMODE viewMode;
   viewMode = COLOR;

   float focalLength = parameters[0];
   float cmosWidth = parameters[1];
   float cmosHeight = parameters[2];
   float halfCMOSWidth = cmosWidth / 2.0;
   float halfCMOSHeight = cmosHeight / 2.0;

   float translationX = parameters[3];
   float translationY = parameters[4];
   float translationZ = parameters[5];
   float rotationMatrixM00 = parameters[6];
   float rotationMatrixM01 = parameters[7];
   float rotationMatrixM02 = parameters[8];
   float rotationMatrixM10 = parameters[9];
   float rotationMatrixM11 = parameters[10];
   float rotationMatrixM12 = parameters[11];
   float rotationMatrixM20 = parameters[12];
   float rotationMatrixM21 = parameters[13];
   float rotationMatrixM22 = parameters[14];

   float principalOffsetXPixels = parameters[15];
   float principalOffsetYPixels = parameters[16];
   float focalLengthPixelsX = parameters[17];
   float focalLengthPixelsY = parameters[18];
   int depthImageWidth = parameters[19];
   int depthImageHeight = parameters[20];
   int colorImageWidth = parameters[21];
   int colorImageHeight = parameters[22];

   bool sinusoidal = parameters[23] > 0.5; //TODO: Create a boolean OpenCL buffer for these binary parameters

   int x = get_global_id(0);
   int y = get_global_id(1);

    // printf("OpenCLKernel -> (x: %d, y: %d)\n", x, y);

   float eyeDepthInMeters = read_imagef(depthImageMeters, (int2) (x, y)).x;

   float zUp3DX = eyeDepthInMeters;
   float zUp3DY = -(x - principalOffsetXPixels) / focalLengthPixelsX * eyeDepthInMeters;
   float zUp3DZ = -(y - principalOffsetYPixels) / focalLengthPixelsY * eyeDepthInMeters;

   float cmosToPixelsX = colorImageWidth / cmosWidth;
   float cmosToPixelsY = colorImageHeight / cmosHeight;

   // Flip because positive yaw is to the left, but image coordinates go to the right
   float yaw = -angle(1.0, 0.0, zUp3DX, zUp3DY);
   double distanceFromSensorCenterX = focalLength * tan(yaw);
   double distanceFromSensorLeftX = distanceFromSensorCenterX + halfCMOSWidth;
   int pixelIndexX = (int) round(distanceFromSensorLeftX * cmosToPixelsX);
   bool pixelInBounds = intervalContains(pixelIndexX, 0, colorImageWidth);

   double pitch = -angle(1.0, 0.0, zUp3DX, zUp3DZ);
   double distanceFromSensorCenterY = focalLength * tan(pitch);
   double distanceFromSensorTopX = distanceFromSensorCenterY + halfCMOSHeight;
   int pixelIndexY = (int) round(distanceFromSensorTopX * cmosToPixelsY);
   pixelInBounds &= intervalContains(pixelIndexY, 0, colorImageHeight);

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

   int color;
    if(viewMode == COLOR)
    {
        if(pixelInBounds)
        {
            uint4 rgba8888Color = read_imageui(colorRGBAImage, (int2) (pixelIndexX, pixelIndexY));
            color = (rgba8888Color.x << 24) | (rgba8888Color.y << 16) | (rgba8888Color.z << 8) | 255;
        }
       else
       {
         color = createRGB((double) worldFramePoint.z, sinusoidal);
//          color = (255 << 24) | (255 << 16) | (255 << 8) | 255; // white is default
       }

    }
    else if(viewMode == DEPTH)
    {
//              float4 rgb = createRGBFromDepth(eyeDepthInMeters, sinusoidal);
//              color = ((int)rgb.x << 24) | ((int)rgb.y << 16) | ((int)rgb.z << 8) | 255;
          color = createRGB((double) eyeDepthInMeters, sinusoidal);
//              printf("%f, %f, %f\n", rgb.x,rgb.y,rgb.z);
    }
    else
    {
//               float4 rgb = createRGBFromZDepth(zUp3DZ, sinusoidal);
//               color = ((int)rgb.x << 24) | ((int)rgb.y << 16) | ((int)rgb.z << 8) | 255;
//               printf("r: %f, g: %f, b: %f\n", rgb.x,rgb.y,rgb.z);
         color = createRGB((double) worldFramePoint.z);
    }

   int pointStartIndex = (depthImageWidth * y + x) * 8;
   
   finalPointFloatBuffer[pointStartIndex]     = worldFramePoint.x;
   finalPointFloatBuffer[pointStartIndex + 1] = worldFramePoint.y;
   finalPointFloatBuffer[pointStartIndex + 2] = worldFramePoint.z;
   finalPointFloatBuffer[pointStartIndex + 3] = color;

    int rInt = (color >> 24) & 0xFF;
    int gInt = (color >> 16) & 0xFF;
    int bInt = (color >> 8) & 0xFF;
    int aInt = color & 0xFF;

    float r = rInt / 255.0;
    float g = gInt / 255.0;
    float b = bInt / 255.0;
    float a = aInt / 255.0;

    finalPointFloatBuffer[pointStartIndex + 3] = r;
    finalPointFloatBuffer[pointStartIndex + 4] = g;
    finalPointFloatBuffer[pointStartIndex + 5] = b;
    finalPointFloatBuffer[pointStartIndex + 6] = a;

    finalPointFloatBuffer[pointStartIndex + 7] = 0.01f;
}
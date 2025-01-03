float4 calculateInterpolatedGradientColorFloat4(double input)
{
   // Using interpolation between key color points
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

int calculateInterpolatedGradientColorInt(double input)
{
   float4 gradientColor = calculateInterpolatedGradientColorFloat4(input);
   int color = ((int) round(gradientColor.x) << 24) | ((int) round(gradientColor.y) << 16) | ((int) round(gradientColor.z) << 8) | 255;
   return color;
}

float4 calculateSinusoidalGradientFloat4(double input)
{
   // maximum depth value
   float m = 3.0f;
   float a = 5.0f * input * M_PI_F / (3.0f * m) + M_PI_F / 2.0f;
   float r = sin(a) * 192.0f + 128.0f;
   float alpha = 255.0f;

   if (r < 0.0f)
      r = 0.0f;
   else if (r > 255.0f)
      r = 255.0f;
      // r = max(0.0f, min(255.0f, r));
   float g = sin(a - 2.0f * M_PI_F / 3.0f) * 192.0f + 128.0f;
   if (g < 0.0f)
      g = 0.0f;
   else if (g > 255.0f)
      g = 255.0f;
      // g = max(0.0f, min(255.0f, g));
   float b = sin(a - 4.0f * M_PI_F / 3.0f) * 192.0f + 128.0f;
   if (b < 0.0f)
      b = 0.0f;
   else if (b > 255.0f)
      b = 255.0f;
      // b = max(0.0f, min(255.0f, b));
   return (float4) (r / 255.0f, g / 255.0f, b / 255.0f, 1.0);
}

float4 calculateGradientColorOptionFloat4(double input, bool sinusoidal)
{
   if (sinusoidal)
   {
      return calculateSinusoidalGradientFloat4(input);
   }
   else
   {
      return calculateInterpolatedGradientColorFloat4(input);
   }
}

int calculateGradientColor(double input, bool sinusoidal)
{
   if (sinusoidal)
   {
      float4 colorFloatRGBA = calculateSinusoidalGradientFloat4(input);
      int color = ((int) round(colorFloatRGBA.x * 255.0f) << 24)
                | ((int) round(colorFloatRGBA.y * 255.0f) << 16)
                | ((int) round(colorFloatRGBA.z * 255.0f) << 8)
                | 255;
      return color;
   }
   else
   {
      return calculateInterpolatedGradientColorInt(input);
   }
}

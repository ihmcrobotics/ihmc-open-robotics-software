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

int calculateGradientColor(double input, bool sinusoidal)
{
   if (sinusoidal)
   {
      // maximum depth value
      float m = 3;
      float PI = 3.141592;
      float a = 5 * input * PI / (3 * m) + PI / 2;
      float r = sin(a) * 192 + 128;
      float alpha = 255;

      if (r < 0)
         r = 0;
      else if (r > 255)
         r = 255;
      //    r=max(0,min(255,r));
      float g = sin(a - 2 * PI / 3) * 192 + 128;
      if (g < 0)
         g = 0;
      else if (g > 255)
         g = 255;
      //    g=max(0,min(255,g));
      float b = sin(a - 4 * PI / 3) * 192 + 128;
      if (b < 0)
         b = 0;
      else if (b > 255)
         b = 255;
      //    b=max(0,min(255,b));
      int color = ((int) round(r) << 24) | ((int) round(g) << 16) | ((int) round(b) << 8) | 255;
      return color;
   }
   else
   {
      return calculateInterpolatedGradientColorInt(input);
   }
}

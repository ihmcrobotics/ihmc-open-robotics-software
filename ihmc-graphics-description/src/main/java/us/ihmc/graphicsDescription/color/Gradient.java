package us.ihmc.graphicsDescription.color;

import java.awt.Color;

/**
 * Class to create color gradients.
 *
 */
public class Gradient
{
   public static Color[] createRainbow(int steps)
   {
      return createMultiGradient(new Color[] { Color.magenta, Color.blue, Color.green, Color.yellow, Color.orange, Color.red }, steps);
   }

   public static Color[] createGradient(Color start, Color end, int steps)
   {
      int startRed = start.getRed();
      int startGreen = start.getGreen();
      int startBlue = start.getBlue();
      int startAlpha = start.getAlpha();

      int endRed = end.getRed();
      int endGreen = end.getGreen();
      int endBlue = end.getBlue();
      int endAlpha = end.getAlpha();

      Color[] gradient = new Color[steps];

      double redRange = (endRed - startRed);
      double greenRange = (endGreen - startGreen);
      double blueRange = (endBlue - startBlue);
      double alphaRange = (endAlpha - startAlpha);
      
      for (int i = 0; i < steps; i++)
      {
         double stepFactor = (double) i / (double) steps; 
         
         gradient[i] = new Color(startRed + (int) (stepFactor * redRange), startGreen + (int) (stepFactor * greenRange), startBlue + (int) (stepFactor * blueRange), startAlpha + (int) (stepFactor * alphaRange));
      }

      return gradient;
   }

   public static Color[] createMultiGradient(Color[] colors, int steps)
   {
      if(colors.length < 2)
      {
         throw new IllegalArgumentException("Need at least 2 colors for a gradient");
      }
      
      
      Color[] gradient = new Color[steps];
      
      int stepsBetween = steps / (colors.length - 1); 
      
      
      int index = 0;
      for (int i = 0; i < colors.length - 1; i++)
      {
         Color[] thisGradient = createGradient(colors[i], colors[i + 1], stepsBetween);
         
         System.arraycopy(thisGradient, 0, gradient, index, stepsBetween);
         index += stepsBetween;
      }
      
      for(; index < steps; index++)
      {
         gradient[index] = colors[colors.length - 1];
      }
      
      return gradient;
   }

}

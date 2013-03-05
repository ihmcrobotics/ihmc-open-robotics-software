package us.ihmc.imageProcessing.ImageFilters;

import java.awt.Color;
import java.util.ArrayList;

import jxl.format.RGB;


public class ColorFilter extends PointFilter
{
   private ArrayList<RGB> colorsToLookFor = new ArrayList<RGB>();

   double threshold = 70;

   public double getThreshold()
   {
      return threshold;
   }


   public void setThreshold(double threshold)
   {
      this.threshold = threshold;
   }


   public ColorFilter()
   {
      canFilterIndexColorModel = true;
   }


   public ColorFilter(ArrayList<RGB> colorsToLookFor)
   {
      canFilterIndexColorModel = true;
      this.colorsToLookFor = colorsToLookFor;
   }

   /**
    * Set the colormap to be used for the filter.
    * @param colormap the colormap
    * @see #getColormap
    */
   public void addColorToLookFor(RGB color)
   {
      colorsToLookFor.add(color);
   }

   public int filterRGB(int x, int y, int rgb)
   {

//    int a = rgb & 0xff000000;

      for (RGB c : colorsToLookFor)
      {
         double colorDiff = colorDist((rgb >> 16) & 0xff, (rgb >> 8) & 0xff, rgb & 0xff, c.getRed(), c.getGreen(), c.getBlue());
         if (colorDiff < threshold)
         {
            return 0;
         }
      }

      return 16777215;
   }


   double colorDist(int r1, int g1, int b1, int r2, int g2, int b2)
   {
      long rmean = ((long) r1 + (long) r2) / 2;
      long r = (long) r1 - (long) r2;
      long g = (long) g1 - (long) g2;
      long b = (long) b1 - (long) b2;

      return Math.sqrt((((512 + rmean) * r * r) >> 8) + 4 * g * g + (((767 - rmean) * b * b) >> 8));
   }




   public String toString()
   {
      return "Colors/Lookup...";
   }

}

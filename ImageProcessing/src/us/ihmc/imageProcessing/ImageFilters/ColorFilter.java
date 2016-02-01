package us.ihmc.imageProcessing.ImageFilters;

import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.util.ArrayList;

import jxl.format.RGB;


public class ColorFilter extends PointFilter
{
   private ArrayList<RGB> colorsToLookFor = new ArrayList<RGB>();

   double threshold = 100;
   private int horizon = 400;
   boolean filterHorizon = false;

   public void setHorizonYLocation(int y)
   {
      horizon = y;
   }

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

  

   public BufferedImage filter(BufferedImage src, BufferedImage dst)
   {
      int width = src.getWidth();
      int height = src.getHeight();
      int type = src.getType();
      WritableRaster srcRaster = src.getRaster();

      if (dst == null)
         dst = createCompatibleDestImage(src, null);
      WritableRaster dstRaster = dst.getRaster();

      setDimensions(width, height);

      int[] inPixels = new int[width];
      for (int y = 0; y < height; y++)
      {
         // We try to avoid calling getRGB on images as it causes them to become unmanaged, causing horrible performance problems.
         if (type == BufferedImage.TYPE_INT_ARGB)
         {
            srcRaster.getDataElements(0, y, width, 1, inPixels);

            for (int x = 0; x < width; x++)
            {
               if (filterHorizon && (y < horizon))
               {
                  inPixels[x] = 16777215;
               }
               else
                  inPixels[x] = filterRGB(x, y, inPixels[x]);
            }

            dstRaster.setDataElements(0, y, width, 1, inPixels);
         }
         else
         {
            src.getRGB(0, y, width, 1, inPixels, 0, width);

            for (int x = 0; x < width; x++)
            {
               if (filterHorizon && (y < horizon))
               {
                  inPixels[x] = 16777215;
               }
               else
                  inPixels[x] = filterRGB(x, y, inPixels[x]);
            }

            dst.setRGB(0, y, width, 1, inPixels, 0, width);
         }
      }

      return dst;
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

   public void filterHorizon(boolean filterHorizon)
   {
      this.filterHorizon = filterHorizon;
   }

}

package us.ihmc.gdx.logging;

public class FFMPEGPixel
{
   private final double R;
   private final double G;
   private final double B;

   private final boolean storedAsYUV;
   private final double Y;
   private final double U;
   private final double V;

   public FFMPEGPixel(double R, double G, double B)
   {
      this(R, G, B, false);
   }

   public double getR()
   {
      return R;
   }

   public double getG()
   {
      return G;
   }

   public double getB()
   {
      return B;
   }

   public double getY()
   {
      return Y;
   }

   public double getU()
   {
      return U;
   }

   public double getV()
   {
      return V;
   }

   /**
    * @param valueX Either R (RGB) or Y (YUV) depending on mode
    * @param valueY Either G (RGB) or U (YUV) depending on mode
    * @param valueZ Either B (RGB) or V (YUV) depending on mode
    * @param isYUV  Determines mode
    */
   public FFMPEGPixel(double valueX, double valueY, double valueZ, boolean isYUV)
   {
      storedAsYUV = isYUV;

      //Modified from https://stackoverflow.com/questions/17892346/how-to-convert-rgb-yuv-rgb-both-ways
      if (isYUV)
      {
         R = valueX;
         G = valueY;
         B = valueZ;

         Y = 0.257 * R + 0.504 * G + 0.098 * B + 16;
         U = -0.148 * R - 0.291 * G + 0.439 * B + 128;
         V = 0.439 * R - 0.368 * G - 0.071 * B + 128;
      }
      else
      {
         Y = valueX;
         U = valueY;
         V = valueZ;

         valueX -= 16;
         valueY -= 128;
         valueZ -= 128;

         R = 1.164 * valueX + 1.596 * valueZ;
         G = 1.164 * valueX - 0.392 * valueY - 0.813 * valueZ;
         B = 1.164 * valueX + 2.017 * valueY;
      }
   }
}

package us.ihmc.robotics.alphaToAlpha;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SmearedSinusoidalTransformation implements AlphaToAlphaFunction
{
   private boolean zeroVelAtStart;
   private double exponent;

   public SmearedSinusoidalTransformation(int originalListSize, int finalListSize, boolean zeroVelAtStart)
   {
      double scaleFactor = ((double) finalListSize) / ((double) originalListSize);
      double fudgeFactor = 0.8;

      exponent = Math.pow(scaleFactor / (Math.PI / 2.0), fudgeFactor);
      this.zeroVelAtStart = zeroVelAtStart;
   }

   public double getAlphaPrime(double alpha)
   {
      double alphaPrime;

      alpha = (alpha > 1.0) ? 1.0 : alpha;
      alpha = (alpha < 0.0) ? 0.0 : alpha;

      alpha = Math.pow(alpha, exponent);

      if (zeroVelAtStart)
      {
         alphaPrime = 1.0 - Math.cos(Math.PI / 2.0 * alpha);
      }
      else
      {
         alphaPrime = Math.sin(Math.PI / 2.0 * alpha);
      }


      return alphaPrime;
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }


   public double getMaxAlpha()
   {
      return 1.0;
   }

   public static void main(String[] args)
   {
      new SmearedSinusoidalTransformation(300, 100, true);    // 2.14286);
   }

}

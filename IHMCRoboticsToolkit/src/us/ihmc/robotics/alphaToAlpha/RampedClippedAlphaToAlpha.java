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
public class RampedClippedAlphaToAlpha implements AlphaToAlphaFunction
{
   private final double upRampStart;
   private final double upRampEnd;
   private final double downRampStart;
   private final double downRampEnd;

   public RampedClippedAlphaToAlpha(double upRampStart, double upRampEnd, double downRampStart, double downRampEnd)
   {
      this.upRampStart = upRampStart;
      this.upRampEnd = upRampEnd;
      this.downRampStart = downRampStart;
      this.downRampEnd = downRampEnd;
   }

   public double getAlphaPrime(double alpha)
   {
      if (alpha < upRampStart)
         return 0.0;
      else if (alpha < upRampEnd)
         return (alpha - upRampStart) / (upRampEnd - upRampStart);
      else if (alpha < downRampStart)
         return 1.0;
      else if (alpha < downRampEnd)
         return 1.0 - (alpha - downRampStart) / (downRampEnd - downRampStart);
      else
         return 0.0;
   }

   public double getMaxAlpha()
   {
      return 1.0;
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }


   public static void main(String[] args)
   {
      new RampedClippedAlphaToAlpha(0.1, 0.2, 0.8, 0.9);
   }
}

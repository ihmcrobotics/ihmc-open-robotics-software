package us.ihmc.robotics.alphaToAlpha;


public class StretchedLinearAlphaToAlphaFunction implements AlphaToAlphaFunction
{
   public StretchedLinearAlphaToAlphaFunction()
   {
   }

   public double getAlphaPrime(double alpha)
   {
      return alpha;
   }

   public double getMaxAlpha()
   {
      return 1.0;
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      return 1.0;
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      return 0.0;
   }

}

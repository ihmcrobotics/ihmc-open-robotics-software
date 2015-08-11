package us.ihmc.robotics.alphaToAlpha;

public interface AlphaToAlphaFunction
{
   public double getAlphaPrime(double alpha);

   public double getMaxAlpha();

   public double getDerivativeAtAlpha(double alpha);

   public double getSecondDerivativeAtAlpha(double alpha);

}

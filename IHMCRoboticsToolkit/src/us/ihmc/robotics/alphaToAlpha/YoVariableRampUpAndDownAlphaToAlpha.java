package us.ihmc.robotics.alphaToAlpha;

import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Created by Peter on 9/9/2016.
 */
public class YoVariableRampUpAndDownAlphaToAlpha implements AlphaToAlphaFunction
{
   private final YoDouble startOfRampUp;
   private final YoDouble endOfRamp;
   private final YoDouble startOfRampDown;
   private final YoDouble endOfRampDown;

   /*
      alpha < startOfRampUp => alphaPrime = 0
      startOfRampUp < alpha < endOfRamp => alphaPrime =  (alpha - startOfRampUp)/(endOfRamp - startOfRampUp)
      endOfRamp < alpha < startOfRampDown => alphaPrime = 1
      startOfRampDown < alpha < endOfRampDown => alphaPrime = 1 - (alpha - startOfRampDown)/(endOfRampDown - startOfRampDown)
      endOfRampDown < alpha => alphaPrime = 0

      you must set
      0.0 < startOfRampUp < endOfRamp < startOfRampDown < endOfRampDown < 1.0

      If this above condition is not met, then:
      alphaPrime will alway be ZERO!

    */
   public YoVariableRampUpAndDownAlphaToAlpha(YoDouble startOfRampUp, YoDouble endOfRamp, YoDouble startOfRampDown, YoDouble endOfRampDown)
   {
      this.startOfRampUp = startOfRampUp;
      this.endOfRamp = endOfRamp;
      this.startOfRampDown = startOfRampDown;
      this.endOfRampDown = endOfRampDown;
   }

   @Override public double getAlphaPrime(double alpha)
   {
      if (!areVariablesInIncreasingOrderAndLessThanOne())
         return 0.0;

      if (alpha < startOfRampUp.getDoubleValue())
      {
         return 0.0;
      }
      else if(alpha < endOfRamp.getDoubleValue())
      {
         return (alpha - startOfRampUp.getDoubleValue())/(endOfRamp.getDoubleValue() - startOfRampUp.getDoubleValue());
      }
      else if(alpha < startOfRampDown.getDoubleValue())
      {
         return 1.0;
      }
      else if(alpha < endOfRampDown.getDoubleValue())
      {
         return 1.0 - (alpha - startOfRampDown.getDoubleValue())/(endOfRampDown.getDoubleValue() - startOfRampDown.getDoubleValue());
      }
      else
         return 0.0;

   }

   @Override public double getMaxAlpha()
   {
      return 1.0;
   }

   @Override public double getDerivativeAtAlpha(double alpha)
   {
      return Double.NaN;
   }

   @Override public double getSecondDerivativeAtAlpha(double alpha)
   {
      return Double.NaN;
   }

   private boolean areVariablesInIncreasingOrderAndLessThanOne()
   {
      if (startOfRampUp.getDoubleValue() <= 0.0)
         return false;
      if (endOfRamp.getDoubleValue() <= startOfRampUp.getDoubleValue())
         return false;
      if (startOfRampDown.getDoubleValue() <= endOfRamp.getDoubleValue())
         return false;
      if (endOfRampDown.getDoubleValue() <= startOfRampDown.getDoubleValue())
         return false;
      if (endOfRampDown.getDoubleValue() >= 1.0)
         return false;

      return true;
   }
}

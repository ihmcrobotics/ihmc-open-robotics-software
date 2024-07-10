package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import static us.ihmc.commons.MathTools.clamp;

public class SplitFractionTools
{
   public static double appendSplitFraction(double desiredSplitFraction, double currentSplitFraction, double nominalSplitFraction)
   {
      return combineTwoShifts(desiredSplitFraction, currentSplitFraction, nominalSplitFraction);
   }

   public static double appendWeightDistribution(double desiredWeightDistribution, double currentWeightDistribution, double nominalWeightDistribution)
   {
      return combineTwoShifts(desiredWeightDistribution, currentWeightDistribution, nominalWeightDistribution);
   }

   private static double combineTwoShifts(double desiredShift, double currentShift, double nominalShift)
   {
      //clamping input values, incase we get negative values
      desiredShift = clamp(desiredShift, 0.0, 1);
      currentShift = clamp(currentShift, 0.0, 1);
      nominalShift = clamp(nominalShift, 0.0, 1);

      if (currentShift == -1.0)
         return desiredShift;

      //transfer = desired, default = nominal

      if (desiredShift > nominalShift)
      {
         double desiredPercentShiftForward = (desiredShift - nominalShift) / (1.0 - nominalShift);
         double desiredShiftForward = desiredPercentShiftForward * (1.0 - currentShift);

         //clamping the output to stay within bounds
         return clamp((currentShift + desiredShiftForward), 0.00001, 0.99999);
      }
      else
      {
         double desiredPercentShiftBackward = (nominalShift - desiredShift) / nominalShift;
         double desiredShiftBackward = desiredPercentShiftBackward * currentShift;

         //clamping the output to stay within bounds
         return clamp((currentShift - desiredShiftBackward), 0.00001, 0.99999);
      }
   }
}

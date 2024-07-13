package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import static us.ihmc.commons.MathTools.clamp;

public class SplitFractionTools
{

   private final static double minimumFraction = 1e-5;

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

      if (currentShift == -1.0)
         return desiredShift;

      //clamping input values, incase we get negative values
      desiredShift = clamp(desiredShift, 0.0, 1.0);
      currentShift = clamp(currentShift, 0.0, 1.0);
      nominalShift = clamp(nominalShift, 0.0, 1.0);

      //transfer = desired, default = nominal

      if (desiredShift > nominalShift)
      {
         double desiredPercentShiftForward = (desiredShift - nominalShift) / (1.0 - nominalShift);
         double desiredShiftForward = desiredPercentShiftForward * (1.0 - currentShift);

         //clamping the output to stay within bounds
         return clamp((currentShift + desiredShiftForward), minimumFraction, 1 - minimumFraction);
      }
      else
      {
         double desiredPercentShiftBackward = (nominalShift - desiredShift) / nominalShift;
         double desiredShiftBackward = desiredPercentShiftBackward * currentShift;

         //clamping the output to stay within bounds
         return clamp((currentShift - desiredShiftBackward), minimumFraction, 1 - minimumFraction);
      }
   }
}

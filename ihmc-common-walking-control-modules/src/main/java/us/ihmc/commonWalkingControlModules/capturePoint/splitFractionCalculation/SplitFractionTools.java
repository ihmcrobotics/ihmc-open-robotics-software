package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

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
      if (currentShift == -1.0)
         return desiredShift;

      // TODO apply some clamps on the shifts

      if (desiredShift > nominalShift)
      {
         double desiredPercentShiftForward = (desiredShift - nominalShift) / (1.0 - nominalShift);
         double desiredShiftForward = desiredPercentShiftForward * (1.0 - currentShift);

         // TODO clamp the output
         return currentShift + desiredShiftForward;
      }
      else
      {
         double desiredPercentShiftBackward = (nominalShift - desiredShift) / nominalShift;
         double desiredShiftBackward = desiredPercentShiftBackward * currentShift;

         // TODO clamp the output
         return currentShift - desiredShiftBackward;
      }
   }
}

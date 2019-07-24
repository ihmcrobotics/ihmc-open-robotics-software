package us.ihmc.humanoidRobotics.footstep;

import us.ihmc.commons.MathTools;

/**
 * Holds fractions for the execution of a footstep.
 */
public class FootstepShiftFractions
{
   /** The nominal fraction of the swing duration of a footstep as specified in the FootstepData that is spent shifting the CoP from the heel to the toe.*/
   private double swingDurationShiftFraction = Double.NaN;

   /** The nominal fraction of the swing duration of a footstep spent shifting from the heel to the toe as specified in the FootstepData that is spent shifting
    *  the CoP from the heel to the ball.*/
   private double swingSplitFraction = Double.NaN;

   /** The nominal fraction of the transfer duration of a footstep as specified in the FootstepData that is spent shifting the CoP to the midpoint.*/
   private double transferSplitFraction = Double.NaN;

   /** The weight distribution of the CoP midpoint for the transfer duration between the trailing foot (0.0) and leading foot (1.0) */
   private double transferWeightDistribution = Double.NaN;

   public FootstepShiftFractions()
   {
   }

   public FootstepShiftFractions(double swingDurationShiftFraction, double swingSplitFraction, double transferSplitFraction, double transferWeightDistribution)
   {
      setShiftFractions(swingDurationShiftFraction, swingSplitFraction, transferSplitFraction);
      setTransferWeightDistribution(transferWeightDistribution);
   }

   /**
    * Sets the {@link #swingDurationShiftFraction}, {@link #swingSplitFraction}, and {@link #transferSplitFraction} of the footstep.
    */
   public void setShiftFractions(double swingDurationShiftFraction, double swingSplitFraction, double transferSplitFraction)
   {
      this.swingDurationShiftFraction = swingDurationShiftFraction;
      this.swingSplitFraction = swingSplitFraction;
      this.transferSplitFraction = transferSplitFraction;
      if (!MathTools.intervalContains(swingDurationShiftFraction, 0.0, 1.0, false, false))
         throw new RuntimeException("Swing Duration Shift Fraction is " + swingDurationShiftFraction + ", must be between 0.0 and 1.0.");
      if (!MathTools.intervalContains(swingSplitFraction, 0.0, 1.0, false, false))
         throw new RuntimeException("Swing Split Fraction is " + swingSplitFraction + ", must be between 0.0 and 1.0.");
      if (!MathTools.intervalContains(transferSplitFraction, 0.0, 1.0, false, false))
         throw new RuntimeException("Transfer Split Fraction is " + transferSplitFraction + ", must be between 0.0 and 1.0.");
   }

   /**
    * Sets the {@link #transferWeightDistribution} of the footstep.
    */
   public void setTransferWeightDistribution(double transferWeightDistribution)
   {
      this.transferWeightDistribution = transferWeightDistribution;
      if (!MathTools.intervalContains(transferWeightDistribution, 0.0, 1.0))
         throw new RuntimeException("Transfer weight distribution is " + transferWeightDistribution + ", must be between 0.0 and 1.0");
   }

   /**
    * Returns the fraction of the swing duration spent shifting the weight from the heel to the toe.
    */
   public double getSwingDurationShiftFraction()
   {
      return swingDurationShiftFraction;
   }

   /**
    * Returns the fraction of the time spent shifting the weight from the heel to the toe in swing that the weight is being shifted from the heel to the ball.
    */
   public double getSwingSplitFraction()
   {
      return swingSplitFraction;
   }

   /**
    * Returns the fraction of the transfer duration shifting the weight from the trailing foot to the middle of stance.
    */
   public double getTransferSplitFraction()
   {
      return transferSplitFraction;
   }

   /**
    * Returns the distribution of the mid-cop in transfer between the trailing foot and the leading foot.
    */
   public double getTransferWeightDistribution()
   {
      return transferWeightDistribution;
   }

   /**
    * Sets this timing to be a copy of the given one.
    */
   public void set(FootstepShiftFractions other)
   {
      swingDurationShiftFraction = other.swingDurationShiftFraction;
      swingSplitFraction = other.swingSplitFraction;
      transferSplitFraction = other.transferSplitFraction;
      transferWeightDistribution = other.transferWeightDistribution;
   }

   public static FootstepShiftFractions[] createShiftFractions(int numberOfTimings)
   {
      FootstepShiftFractions[] timings = new FootstepShiftFractions[numberOfTimings];
      for (int i = 0; i < numberOfTimings; i++)
      {
         timings[i] = new FootstepShiftFractions();
      }
      return timings;
   }
}

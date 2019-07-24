package us.ihmc.humanoidRobotics.footstep;

/**
 * Holds timings for the execution of a footstep.
 */
public class FootstepShiftFractions
{
   /** The nominal swing duration of a footstep as specified in the FootstepData */
   private double swingDurationShiftFraction = Double.NaN;

   /** The nominal swing duration of a footstep as specified in the FootstepData */
   private double swingSplitFraction = Double.NaN;

   /** The nominal swing duration of a footstep as specified in the FootstepData */
   private double transferSplitFraction = Double.NaN;


   public FootstepShiftFractions()
   {
   }

   public FootstepShiftFractions(double swingDurationShiftFraction, double swingSplitFraction, double transferSplitFraction)
   {
      setShiftFractions(swingDurationShiftFraction, swingSplitFraction, transferSplitFraction);
   }

   /**
    * Sets the {@link #swingDurationShiftFraction}, {@link #swingSplitFraction}, and {@link #transferSplitFraction} of the footstep.
    */
   public void setShiftFractions(double swingDurationShiftFraction, double swingSplitFraction, double transferSplitFraction)
   {
      this.swingDurationShiftFraction = swingDurationShiftFraction;
      this.swingSplitFraction = swingSplitFraction;
      this.transferSplitFraction = transferSplitFraction;
      if (swingDurationShiftFraction < 0.0 || swingSplitFraction < 0.0 || transferSplitFraction < 0.0)
      {
         throw new RuntimeException("Swing and transfer split fractions can not be negative.");
      }
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
    * Sets this timing to be a copy of the given one.
    */
   public void set(FootstepShiftFractions other)
   {
      swingDurationShiftFraction = other.swingDurationShiftFraction;
      swingSplitFraction = other.swingSplitFraction;
      transferSplitFraction = other.transferSplitFraction;
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

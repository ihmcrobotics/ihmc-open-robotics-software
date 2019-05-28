package us.ihmc.humanoidRobotics.footstep;

/**
 * Holds timings for the execution of a footstep.
 */
public class FootstepTiming
{
   /** The nominal swing duration of a footstep as specified in the FootstepData */
   private double swingTime = Double.NaN;

   /** The nominal transfer duration of a footstep as specified in the FootstepData */
   private double transferTime = Double.NaN;

   /** Boolean that tells the controller whether it should attempt achieving absolute timings */
   private boolean hasAbsoluteTime = false;

   /** The absolute start time of the swing is computed by the controller if absolute timings are requested */
   private double swingStartTime = Double.NaN;

   /** The controller time at which the footstep data list starts executing if absolute timings are requested */
   private double executionStartTime = Double.NaN;

   public FootstepTiming()
   {
   }

   public FootstepTiming(double swingTime, double transferTime)
   {
      setTimings(swingTime, transferTime);
   }

   /**
    * Sets the {@link #swingTime} and {@link #transferTime} of the footstep.
    */
   public void setTimings(double swingTime, double transferTime)
   {
      this.swingTime = swingTime;
      this.transferTime = transferTime;
      if (transferTime < 0.0 || swingTime < 0.0)
      {
         throw new RuntimeException("Swing and transfer duration can not be negative.");
      }
   }

   /**
    * Sets the {@link #transferTime} of the footstep.
    */
   public void setTransferTime(double transferTime)
   {
      this.transferTime = transferTime;
      if (transferTime < 0.0)
      {
         throw new RuntimeException("Transfer duration can not be negative.");
      }
   }

   /**
    * Returns the time from swing foot lift-off to touch-down.
    */
   public double getSwingTime()
   {
      return swingTime;
   }

   /**
    * Returns the time to transfer the weight to the stance foot before the step is taken.
    */
   public double getTransferTime()
   {
      return transferTime;
   }

   /**
    * Returns the sum of {@link #swingTime} and {@link #transferTime}. This is the total time the step takes from
    * beginning of transferring weight to the stance foot to the touch-down of the swing foot.
    */
   public double getStepTime()
   {
      return swingTime + transferTime;
   }
   

   /**
    * Returns true if the footstep has an absolute timing requirement.
    */
   public boolean hasAbsoluteTime()
   {
      return hasAbsoluteTime;
   }

   /**
    * Sets the timing requirements for this footstep. The swingStartTime is with respect to the executionStartTime.
    */
   public void setAbsoluteTime(double swingStartTime, double executionStartTime)
   {
      hasAbsoluteTime = true;
      this.swingStartTime = swingStartTime;
      this.executionStartTime = executionStartTime;
   }

   /**
    * Removed all absolute timing requirements of this footstep.
    */
   public void removeAbsoluteTime()
   {
      hasAbsoluteTime = false;
      this.swingStartTime = Double.NaN;
      this.executionStartTime = Double.NaN;
   }

   /**
    * If absolute footstep timing is requested this will return the time at which the swing needs to start with respect
    * to the start time of the execution of the footstep plan that this footstep is part of.
    */
   public double getSwingStartTime()
   {
      return swingStartTime;
   }

   /**
    * This returns the time at which the footstep plan that this footstep is part of was started to execute. The value
    * is the reference for absolute footstep timings.
    */
   public double getExecutionStartTime()
   {
      return executionStartTime;
   }

   /**
    * Sets this timing to be a copy of the given one.
    */
   public void set(FootstepTiming other)
   {
      swingTime = other.swingTime;
      transferTime = other.transferTime;
      hasAbsoluteTime = other.hasAbsoluteTime;
      swingStartTime = other.swingStartTime;
      executionStartTime = other.executionStartTime;
   }

   public static FootstepTiming[] createTimings(int numberOfTimings)
   {
      FootstepTiming[] timings = new FootstepTiming[numberOfTimings];
      for (int i = 0; i < numberOfTimings; i++)
      {
         timings[i] = new FootstepTiming();
      }
      return timings;
   }
}

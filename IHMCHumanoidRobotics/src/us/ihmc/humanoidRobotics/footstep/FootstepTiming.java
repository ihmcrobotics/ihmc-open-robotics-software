package us.ihmc.humanoidRobotics.footstep;

public class FootstepTiming
{
   private double swingTime = Double.NaN;
   private double transferTime = Double.NaN;
   private boolean hasAbsoluteTime = false;
   private double swingStartTime = Double.NaN;
   private double executionStartTime = Double.NaN;

   public FootstepTiming()
   {
   }

   public FootstepTiming(double swingTime, double transferTime)
   {
      setTimings(swingTime, transferTime);
   }

   public void setTimings(double swingTime, double transferTime)
   {
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }

   public double getSwingTime()
   {
      return swingTime;
   }

   public double getTransferTime()
   {
      return transferTime;
   }

   public double getStepTime()
   {
      return swingTime + transferTime;
   }

   public boolean hasAbsoluteTime()
   {
      return hasAbsoluteTime;
   }

   public void setAbsoluteTime(double swingStartTime, double executionStartTime)
   {
      hasAbsoluteTime = true;
      this.swingStartTime = swingStartTime;
      this.executionStartTime = executionStartTime;
   }

   public void removeAbsoluteTime()
   {
      hasAbsoluteTime = false;
      this.swingStartTime = Double.NaN;
   }

   public double getSwingStartTime()
   {
      return swingStartTime;
   }

   public double getExecutionStartTime()
   {
      return executionStartTime;
   }

   public void set(FootstepTiming other)
   {
      swingTime = other.swingTime;
      transferTime = other.transferTime;
      hasAbsoluteTime = other.hasAbsoluteTime;
      swingStartTime = other.swingStartTime;
      executionStartTime = other.executionStartTime;
   }
}

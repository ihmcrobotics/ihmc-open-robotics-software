package us.ihmc.commonWalkingControlModules.trajectories;

public class ConstantSwingTimeCalculator implements SwingTimeCalculator
{
   private final double swingTime;
   
   public ConstantSwingTimeCalculator(double swingTime)
   {
      this.swingTime = swingTime;
   }
   
   public double getSwingTime(double stepLength)
   {
      return swingTime;
   }

}

package us.ihmc.commonWalkingControlModules.trajectories;

public class ConstantSwingTimeCalculator implements SwingTimeCalculator
{

   private final double swingTime;
   
   public ConstantSwingTimeCalculator(double swingTime)
   {
      this.swingTime = swingTime;
   }
   
   @Override
   public void setSwingTime(double stepLength)
   {
   }

   @Override
   public double getSwingTime()
   {
      return swingTime;
   }

}

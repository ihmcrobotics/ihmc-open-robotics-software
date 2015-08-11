package us.ihmc.robotics.trajectories.providers;


public class ConstantDoubleProvider implements DoubleProvider
{
   private final double value;

   public ConstantDoubleProvider(double value)
   {
      this.value = value;
   }

   public double getValue()
   {
      return value;
   }

}

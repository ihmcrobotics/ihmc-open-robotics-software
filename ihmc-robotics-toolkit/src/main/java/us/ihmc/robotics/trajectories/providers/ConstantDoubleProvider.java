package us.ihmc.robotics.trajectories.providers;

import us.ihmc.yoVariables.providers.DoubleProvider;

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

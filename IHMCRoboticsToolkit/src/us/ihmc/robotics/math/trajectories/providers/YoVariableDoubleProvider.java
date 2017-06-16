package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class YoVariableDoubleProvider implements DoubleProvider
{
   private final YoDouble value;

   public YoVariableDoubleProvider(String name, YoVariableRegistry registry)
   {
      value = new YoDouble(name, registry);
   }
   
   public YoVariableDoubleProvider(YoDouble value)
   {
      this.value = value;
   }

   public double getValue()
   {
      return value.getDoubleValue();
   }
   
   public void set(double value)
   {
      this.value.set(value);
   }
}

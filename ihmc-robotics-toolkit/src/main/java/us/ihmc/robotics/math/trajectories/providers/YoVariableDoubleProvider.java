package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;


public class YoVariableDoubleProvider implements DoubleProvider
{
   private final YoDouble value;

   public YoVariableDoubleProvider(String name, YoRegistry registry)
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

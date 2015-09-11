package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class YoVariableDoubleProvider implements DoubleProvider
{
   private final DoubleYoVariable value;

   public YoVariableDoubleProvider(String name, YoVariableRegistry registry)
   {
      value = new DoubleYoVariable(name, registry);
   }
   
   public YoVariableDoubleProvider(DoubleYoVariable value)
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

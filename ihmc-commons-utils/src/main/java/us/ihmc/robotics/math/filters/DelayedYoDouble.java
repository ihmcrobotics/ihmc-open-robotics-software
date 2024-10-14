package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is a class that introduces purposeful delay between the passed in variable and the output of this variable. This is useful to capture latency in a
 * process. The length of the delay is set by a specific number of ticks.
 */
public class DelayedYoDouble extends YoDouble
{
   private final DoubleProvider variableToDelay;

   private final YoDouble[] previousYoDouble;

   public DelayedYoDouble(String name, String description, DoubleProvider variableToDelay, int ticksToDelay, YoRegistry registry)
   {
      super(name, description, registry);

      this.variableToDelay = variableToDelay;
      previousYoDouble = new YoDouble[ticksToDelay];

      for (int i = 0; i < ticksToDelay; i++)
      {
         previousYoDouble[i] = new YoDouble(name + "_previous" + i, registry);
         previousYoDouble[i].set(variableToDelay.getValue());
      }

      this.set(variableToDelay.getValue());
   }

   public void update()
   {
      if (previousYoDouble.length == 0)
      {
         this.set(variableToDelay.getValue());
         return;
      }

      this.set(previousYoDouble[0].getValue());

      for (int i = 0; i < previousYoDouble.length - 1; i++)
      {
         previousYoDouble[i].set(previousYoDouble[i + 1].getValue());
      }

      previousYoDouble[previousYoDouble.length - 1].set(variableToDelay.getValue());
   }

   public void reset()
   {
      for (YoDouble var : previousYoDouble)
      {
         var.set(variableToDelay.getValue());
      }
      this.set(variableToDelay.getValue());
   }
}

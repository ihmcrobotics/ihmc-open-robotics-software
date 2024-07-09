package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DelayedYoDouble extends YoDouble
{
   private final DoubleProvider variableToDelay;

   private final YoDouble[] previousYoVariables;

   public DelayedYoDouble(String name, String description, DoubleProvider variableToDelay, int ticksToDelay, YoRegistry registry)
   {
      super(name, description, registry);

      this.variableToDelay = variableToDelay;
      previousYoVariables = new YoDouble[ticksToDelay];

      for (int i = 0; i < ticksToDelay; i++)
      {
         previousYoVariables[i] = new YoDouble(name + "_previous" + i, registry);
         previousYoVariables[i].set(variableToDelay.getValue());
      }

      this.set(variableToDelay.getValue());
   }

   public void update()
   {
      if (previousYoVariables.length == 0)
      {
         this.set(variableToDelay.getValue());
         return;
      }

      this.set(previousYoVariables[0].getValue());

      for (int i = 0; i < previousYoVariables.length - 1; i++)
      {
         previousYoVariables[i].set(previousYoVariables[i + 1].getValue());
      }

      previousYoVariables[previousYoVariables.length - 1].set(variableToDelay.getValue());
   }

   public void reset()
   {
      for (YoDouble var : previousYoVariables)
      {
         var.set(variableToDelay.getValue());
      }
      this.set(variableToDelay.getValue());
   }
}

package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DelayedYoDouble extends YoDouble
{
   private final YoDouble variableToDelay;

   private final YoDouble[] previousYoVariables;

   public DelayedYoDouble(String name, String description, YoDouble variableToDelay, int ticksToDelay, YoVariableRegistry registry)
   {
      super(name, description, registry);

      this.variableToDelay = variableToDelay;
      previousYoVariables = new YoDouble[ticksToDelay];

      for (int i = 0; i < ticksToDelay; i++)
      {
         previousYoVariables[i] = new YoDouble(name + "_previous" + i, registry);
         previousYoVariables[i].set(variableToDelay.getDoubleValue());
      }

      this.set(variableToDelay.getDoubleValue());
   }

   public void update()
   {
      if (previousYoVariables.length == 0)
      {
         this.set(variableToDelay.getDoubleValue());
         return;
      }

      this.set(previousYoVariables[0].getDoubleValue());

      for (int i = 0; i < previousYoVariables.length - 1; i++)
      {
         previousYoVariables[i].set(previousYoVariables[i + 1].getDoubleValue());
      }

      previousYoVariables[previousYoVariables.length - 1].set(variableToDelay.getDoubleValue());
   }

   public void reset()
   {
      for (YoDouble var : previousYoVariables)
      {
         var.set(variableToDelay.getDoubleValue());
      }
      this.set(variableToDelay.getDoubleValue());
   }
}

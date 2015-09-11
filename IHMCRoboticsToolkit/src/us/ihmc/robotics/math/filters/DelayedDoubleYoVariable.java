package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DelayedDoubleYoVariable extends DoubleYoVariable
{
   private final DoubleYoVariable variableToDelay;

   private final DoubleYoVariable[] previousYoVariables;

   public DelayedDoubleYoVariable(String name, String description, DoubleYoVariable variableToDelay, int ticksToDelay, YoVariableRegistry registry)
   {
      super(name, description, registry);

      this.variableToDelay = variableToDelay;
      previousYoVariables = new DoubleYoVariable[ticksToDelay];

      for (int i = 0; i < ticksToDelay; i++)
      {
         previousYoVariables[i] = new DoubleYoVariable(name + "_previous" + i, registry);
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
      for (DoubleYoVariable var : previousYoVariables)
      {
         var.set(variableToDelay.getDoubleValue());
      }
      this.set(variableToDelay.getDoubleValue());
   }
}

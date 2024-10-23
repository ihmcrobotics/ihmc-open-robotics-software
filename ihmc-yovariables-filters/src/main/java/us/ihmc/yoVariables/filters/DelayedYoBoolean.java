package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * This is a class that introduces purposeful delay between the passed in variable and the output of this variable. This is useful to capture latency in a
 * process. The length of the delay is set by a specific number of ticks.
 */
public class DelayedYoBoolean extends YoBoolean
{
   private final YoBoolean variableToDelay;

   private final YoBoolean[] previousYoVariables;

   public DelayedYoBoolean(String name, String description, YoBoolean variableToDelay, int ticksToDelay, YoRegistry registry)
   {
      super(name, description, registry);

      this.variableToDelay = variableToDelay;
      previousYoVariables = new YoBoolean[ticksToDelay];

      for (int i = 0; i < ticksToDelay; i++)
      {
         previousYoVariables[i] = new YoBoolean(name + "_previous" + i, registry);
         previousYoVariables[i].set(variableToDelay.getBooleanValue());
      }

      this.set(variableToDelay.getBooleanValue());
   }

   public void update()
   {
      if (previousYoVariables.length == 0)
      {
         this.set(variableToDelay.getBooleanValue());
         return;
      }

      this.set(previousYoVariables[0].getBooleanValue());

      for (int i = 0; i < previousYoVariables.length - 1; i++)
      {
         previousYoVariables[i].set(previousYoVariables[i + 1].getBooleanValue());
      }

      previousYoVariables[previousYoVariables.length - 1].set(variableToDelay.getBooleanValue());
   }

   public void reset()
   {
      for (YoBoolean var : previousYoVariables)
      {
         var.set(variableToDelay.getBooleanValue());
      }
      this.set(variableToDelay.getBooleanValue());
   }

   void getInternalState(String inputString, Boolean ifDebug)
   {
      if (!ifDebug)
         return;

      String string = inputString + "\nvalue = " + this.getBooleanValue() + "\n";
      for (int i = 0; i < previousYoVariables.length; i++)
      {
         string = string + i + " = " + previousYoVariables[i].getBooleanValue() + "\n";
      }
      string = string + "variableToDelay = " + variableToDelay.getBooleanValue() + "\n";
      System.out.println(string);
   }
}
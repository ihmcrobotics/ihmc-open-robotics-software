package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class DelayedYoBoolean extends YoBoolean
{
   private final YoBoolean variableToDelay;

   private final YoBoolean[] previousYoVariables;

   public DelayedYoBoolean(String name, String description, YoBoolean variableToDelay, int ticksToDelay, YoVariableRegistry registry)
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
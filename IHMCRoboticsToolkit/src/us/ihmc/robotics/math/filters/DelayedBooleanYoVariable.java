package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class DelayedBooleanYoVariable extends BooleanYoVariable
{
   private final BooleanYoVariable variableToDelay;

   private final BooleanYoVariable[] previousYoVariables;

   public DelayedBooleanYoVariable(String name, String description, BooleanYoVariable variableToDelay, int ticksToDelay, YoVariableRegistry registry)
   {
      super(name, description, registry);

      this.variableToDelay = variableToDelay;
      previousYoVariables = new BooleanYoVariable[ticksToDelay];

      for (int i = 0; i < ticksToDelay; i++)
      {
         previousYoVariables[i] = new BooleanYoVariable(name + "_previous" + i, registry);
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
      for (BooleanYoVariable var : previousYoVariables)
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
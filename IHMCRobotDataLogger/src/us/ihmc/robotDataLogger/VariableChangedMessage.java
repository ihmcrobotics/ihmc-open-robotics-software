package us.ihmc.robotDataLogger;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class VariableChangedMessage
{
   private YoVariable variable;
   private double val = -1;

   public VariableChangedMessage()
   {
   }

   public YoVariable getVariable()
   {
      return variable;
   }

   public void setVariable(YoVariable variable)
   {
      this.variable = variable;
   }

   public void setVal(double val)
   {
      this.val = val;
   }

   public double getVal()
   {
      return val;
   }

   public static class Builder implements us.ihmc.concurrent.Builder<VariableChangedMessage>
   {
      @Override
      public VariableChangedMessage newInstance()
      {
         return new VariableChangedMessage();
      }
   }
}
package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class IntGlobalParameter extends GlobalParameter
{
   public IntGlobalParameter(String name, String description, int value, GlobalParameterChangedListener listener)
   {
      super(listener);

      yoVariable = new IntegerYoVariable(name, description, registry);
      ((IntegerYoVariable)yoVariable).set(value);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   protected IntGlobalParameter(String name, String description, GlobalParameter[] parents, GlobalParameterChangedListener listener)
   {
      super(parents, listener);

      yoVariable = new IntegerYoVariable(name, description, registry);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   public int getValue()
   {
      return ((IntegerYoVariable)yoVariable).getIntegerValue();
   }

   protected int getNumberOfCharactersForDisplay()
   {
      return Integer.toString(Integer.MIN_VALUE).length();
   }

   public void set(int value)
   {
      set(value, "");
   }

   public void set(int value, String comment)
   {
      verifyNoParents();
      setIntegerValue(value, comment);
   }

   public void setOnlyIfChange(int value)
   {
      setOnlyIfChange(value, "");
   }

   public void setOnlyIfChange(int value, String comment)
   {
      verifyNoParents();

      if (value == ((IntegerYoVariable)yoVariable).getIntegerValue())
         return;
      else
         set(value, comment);
   }


   @Override
   public String getValueInStringFormat()
   {
      String ret = Integer.toString(((IntegerYoVariable)yoVariable).getIntegerValue());

      return padWithSpaces(ret, numberOfCharactersForDisplay);
   }



   private void setIntegerValue(int newValue, String comment)
   {
      int previousValue = ((IntegerYoVariable)yoVariable).getIntegerValue();
      ((IntegerYoVariable)yoVariable).set(newValue);

      if (changedListener != null)
      {
         changedListener.integerValueChanged(this, comment, previousValue, newValue);
      }

      updateChildren(comment);
   }




}

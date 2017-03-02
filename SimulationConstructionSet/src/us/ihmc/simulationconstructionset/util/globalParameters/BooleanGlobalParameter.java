package us.ihmc.simulationconstructionset.util.globalParameters;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class BooleanGlobalParameter extends GlobalParameter
{
   public BooleanGlobalParameter(String name, String description, boolean value, GlobalParameterChangedListener listener)
   {
      super(listener);

      yoVariable = new BooleanYoVariable(name, description, registry);
      ((BooleanYoVariable)yoVariable).set(value);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   protected BooleanGlobalParameter(String name, String description, GlobalParameter[] parents, GlobalParameterChangedListener listener)
   {
      super(parents, listener);

      yoVariable = new BooleanYoVariable(name, description, registry);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   protected int getNumberOfCharactersForDisplay()
   {
      return "false".length();
   }

   public boolean getValue()
   {
      return ((BooleanYoVariable)yoVariable).getBooleanValue();
   }


   public void set(boolean value)
   {
      set(value, "");
   }

   public void set(boolean value, String comment)
   {
      verifyNoParents();
      setBooleanValue(value, comment);
   }

   public void setOnlyIfChange(boolean value)
   {
      setOnlyIfChange(value, "");
   }

   public void setOnlyIfChange(boolean value, String comment)
   {
      verifyNoParents();

      // check if the value is the same as the current value
      if (value == ((BooleanYoVariable)yoVariable).getBooleanValue())
         return;
      else
         setBooleanValue(value, comment);
   }


   @Override
   public String getValueInStringFormat()
   {
      String ret = Boolean.toString(((BooleanYoVariable)yoVariable).getBooleanValue());

      return padWithSpaces(ret, numberOfCharactersForDisplay);
   }



   protected void setBooleanValue(boolean newValue, String comment)
   {
      boolean previousValue = ((BooleanYoVariable)yoVariable).getBooleanValue();
      ((BooleanYoVariable)yoVariable).set(newValue);

      if (changedListener != null)
      {
         changedListener.booleanValueChanged(this, comment, previousValue, newValue);
      }

      updateChildren(comment);
   }




}

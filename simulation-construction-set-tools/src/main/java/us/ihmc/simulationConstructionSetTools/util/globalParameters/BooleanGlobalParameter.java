package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import us.ihmc.yoVariables.variable.YoBoolean;

public class BooleanGlobalParameter extends GlobalParameter
{
   public BooleanGlobalParameter(String name, String description, boolean value, GlobalParameterChangedListener listener)
   {
      super(listener);

      yoVariable = new YoBoolean(name, description, registry);
      ((YoBoolean)yoVariable).set(value);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   protected BooleanGlobalParameter(String name, String description, GlobalParameter[] parents, GlobalParameterChangedListener listener)
   {
      super(parents, listener);

      yoVariable = new YoBoolean(name, description, registry);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   protected int getNumberOfCharactersForDisplay()
   {
      return "false".length();
   }

   public boolean getValue()
   {
      return ((YoBoolean)yoVariable).getBooleanValue();
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
      if (value == ((YoBoolean)yoVariable).getBooleanValue())
         return;
      else
         setBooleanValue(value, comment);
   }


   @Override
   public String getValueInStringFormat()
   {
      String ret = Boolean.toString(((YoBoolean)yoVariable).getBooleanValue());

      return padWithSpaces(ret, numberOfCharactersForDisplay);
   }



   protected void setBooleanValue(boolean newValue, String comment)
   {
      boolean previousValue = ((YoBoolean)yoVariable).getBooleanValue();
      ((YoBoolean)yoVariable).set(newValue);

      if (changedListener != null)
      {
         changedListener.booleanValueChanged(this, comment, previousValue, newValue);
      }

      updateChildren(comment);
   }




}

package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DoubleGlobalParameter extends GlobalParameter
{
   public DoubleGlobalParameter(String name, String description, double value, GlobalParameterChangedListener listener)
   {
      super(listener);

      yoVariable = new DoubleYoVariable(name, description, registry);
      ((DoubleYoVariable)yoVariable).set(value);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   protected DoubleGlobalParameter(String name, String description, GlobalParameter[] parents, GlobalParameterChangedListener listener)
   {
      super(parents, listener);

      yoVariable = new DoubleYoVariable(name, description,registry);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   public double getValue()
   {
      return  ((DoubleYoVariable)yoVariable).getDoubleValue();
   }

   protected int getNumberOfCharactersForDisplay()
   {
      return 9;
   }

   public void setOnlyIfChange(double value)
   {
      setOnlyIfChange(value, "");
   }

   public void setOnlyIfChange(double value, String comment)
   {
      verifyNoParents();

      // check if the value is the same as the current value
      if (value ==  ((DoubleYoVariable)yoVariable).getDoubleValue())
         return;
      else
         setDoubleValue(value, comment);
   }

   public void set(double value)
   {
      set(value, "");
   }

   public void set(double value, String comment)
   {
      verifyNoParents();
      setDoubleValue(value, comment);
   }

   @Override
   public String getValueInStringFormat()
   {
      String ret = Double.toString( ((DoubleYoVariable)yoVariable).getDoubleValue());

      return padWithSpaces(ret, numberOfCharactersForDisplay);
   }



   protected void setDoubleValue(double newValue, String comment)
   {
//    checkIfThereAreParents();

      double previousValue =  ((DoubleYoVariable)yoVariable).getDoubleValue();
      ((DoubleYoVariable)yoVariable).set(newValue);

      if (changedListener != null)
      {
         changedListener.doubleValueChanged(this, comment, previousValue, newValue);
      }

      updateChildren(comment);
   }
}

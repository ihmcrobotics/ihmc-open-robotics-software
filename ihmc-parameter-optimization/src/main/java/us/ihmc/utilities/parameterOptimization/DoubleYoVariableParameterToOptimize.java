package us.ihmc.utilities.parameterOptimization;

import us.ihmc.yoVariables.variable.YoDouble;

public class DoubleYoVariableParameterToOptimize extends DoubleParameterToOptimize
{
   private final YoDouble yoVariable;
   
   public DoubleYoVariableParameterToOptimize(double min, double max, YoDouble yoVariable, ListOfParametersToOptimize listOfParametersToOptimize)
   {
      super(yoVariable.getName(), min, max, listOfParametersToOptimize);
      this.yoVariable = yoVariable;
      
      this.setCurrentValue(yoVariable.getDoubleValue());
//      System.out.println("Creating DoubleYoVariableParameterToOptimize. min = " + min + ", max = " + max + "currentValue = " + this.getCurrentValue());
   }
   
   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      super.setCurrentValueGivenZeroToOne(zeroToOne);
      setParameterValueIntoYoVariable();
   }

   public void setCurrentValue(double newValue)
   {
      super.setCurrentValue(newValue);
      setParameterValueIntoYoVariable();
   }
   
   public void setCurrentValue(ParameterToOptimize parameterToOptimize)
   {
      super.setCurrentValue(parameterToOptimize);
      setParameterValueIntoYoVariable();
   }
   
   
   private void setParameterValueIntoYoVariable()
   {
      this.yoVariable.set(super.getCurrentValue());
   }

}

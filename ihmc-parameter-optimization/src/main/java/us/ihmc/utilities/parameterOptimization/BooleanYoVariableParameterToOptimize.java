package us.ihmc.utilities.parameterOptimization;

import us.ihmc.yoVariables.variable.YoBoolean;

public class BooleanYoVariableParameterToOptimize extends BooleanParameterToOptimize
{
   private final YoBoolean yoVariable;
   
   public BooleanYoVariableParameterToOptimize(YoBoolean yoVariable, ListOfParametersToOptimize listOfParametersToOptimize)
   {
      super(yoVariable.getName(), listOfParametersToOptimize);
      this.yoVariable = yoVariable;
   }
   
   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      super.setCurrentValueGivenZeroToOne(zeroToOne);
      yoVariable.set(this.getCurrentValue());
   }
   
   public void setCurrentValue(boolean newValue)
   {
      super.setCurrentValue(newValue);
      yoVariable.set(newValue);
   }

}

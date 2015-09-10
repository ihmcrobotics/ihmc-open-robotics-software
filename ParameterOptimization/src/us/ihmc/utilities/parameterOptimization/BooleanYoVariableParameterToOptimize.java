package us.ihmc.utilities.parameterOptimization;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class BooleanYoVariableParameterToOptimize extends BooleanParameterToOptimize
{
   private final BooleanYoVariable yoVariable;
   
   public BooleanYoVariableParameterToOptimize(BooleanYoVariable yoVariable, ListOfParametersToOptimize listOfParametersToOptimize)
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

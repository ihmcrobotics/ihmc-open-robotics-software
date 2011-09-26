package us.ihmc.util.parameterOptimization;

import com.yobotics.simulationconstructionset.BooleanYoVariable;

public class BooleanYoVariableParameterToOptimize extends BooleanParameterToOptimize
{
   private final BooleanYoVariable yoVariable;
   
   public BooleanYoVariableParameterToOptimize(BooleanYoVariable yoVariable)
   {
      super();
      this.yoVariable = yoVariable;
   }
   
   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      super.setCurrentValueGivenZeroToOne(zeroToOne);
      yoVariable.set(this.getCurrentValue());
   }

}

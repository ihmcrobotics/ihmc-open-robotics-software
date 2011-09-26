package us.ihmc.util.parameterOptimization;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public class DoubleYoVariableParameterToOptimize extends DoubleParameterToOptimize
{
   private final DoubleYoVariable yoVariable;
   
   public DoubleYoVariableParameterToOptimize(double min, double max, DoubleYoVariable yoVariable)
   {
      super(min, max);
      this.yoVariable = yoVariable;
   }
   
   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      super.setCurrentValueGivenZeroToOne(zeroToOne);
      yoVariable.set(this.getCurrentValue());
   }

}

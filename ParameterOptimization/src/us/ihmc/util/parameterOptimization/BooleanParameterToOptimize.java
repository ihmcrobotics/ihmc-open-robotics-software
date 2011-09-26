package us.ihmc.util.parameterOptimization;


public class BooleanParameterToOptimize implements ParameterToOptimize
{   
   private boolean currentValue;
   
   public boolean getValueGivenZeroToOne(double zeroToOne)
   {
      return (zeroToOne > 0.5);
   }

   public ParameterToOptimizeType getType()
   {
      return ParameterToOptimizeType.BOOLEAN;
   }

   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      currentValue = getValueGivenZeroToOne(zeroToOne);
   }
   
   public boolean getCurrentValue()
   {
      return currentValue;
   }

   public void setCurrentValue(ParameterToOptimize parameterToOptimize)
   {
      setCurrentValue(((BooleanParameterToOptimize) parameterToOptimize).getCurrentValue());
   }
   
   public void setCurrentValue(boolean currentValue)
   {
      this.currentValue = currentValue;
   }
}

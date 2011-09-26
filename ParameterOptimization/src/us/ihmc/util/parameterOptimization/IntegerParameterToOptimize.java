package us.ihmc.util.parameterOptimization;

public class IntegerParameterToOptimize implements ParameterToOptimize
{
   private final int min, max;
   private int currentValue;
   
   public IntegerParameterToOptimize(int min, int max)
   {
      this.min = min;
      this.max = max;
   }
   
   public int getValueGivenZeroToOne(double zeroToOne)
   {
      return (int) (Math.round(((double) min) + zeroToOne * ((double) (max - min))));
   }

   public ParameterToOptimizeType getType()
   {
      return ParameterToOptimizeType.INTEGER;
   }

   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      currentValue = getValueGivenZeroToOne(zeroToOne);
   }
   
   public int getCurrentValue()
   {
      return currentValue;
   }
   
   public void setCurrentValue(ParameterToOptimize parameterToOptimize)
   {
      setCurrentValue(((IntegerParameterToOptimize) parameterToOptimize).getCurrentValue());
   }
   
   public void setCurrentValue(int newValue)
   {
      this.currentValue = newValue;
   }
   
   
}

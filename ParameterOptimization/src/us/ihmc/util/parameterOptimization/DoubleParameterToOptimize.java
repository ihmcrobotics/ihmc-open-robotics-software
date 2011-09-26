package us.ihmc.util.parameterOptimization;

public class DoubleParameterToOptimize implements ParameterToOptimize
{
   private final double min, max;
   private double currentValue;
   
   public DoubleParameterToOptimize(double min, double max)
   {
      this.min = min; 
      this.max = max;
   }
   
   public double getValueGivenZeroToOne(double zeroToOne)
   {
      return min + zeroToOne * (max - min);
   }

   public ParameterToOptimizeType getType()
   {
      return ParameterToOptimizeType.DOUBLE;
   }

   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      currentValue = getValueGivenZeroToOne(zeroToOne);
   }
   
   public double getCurrentValue()
   {
      return currentValue;
   }
}

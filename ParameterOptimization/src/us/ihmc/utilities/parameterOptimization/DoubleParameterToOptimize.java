package us.ihmc.utilities.parameterOptimization;

public class DoubleParameterToOptimize extends ParameterToOptimize
{
   private final double min, max;
   private final int bitsOfResolution;

   private double currentValue;

   public DoubleParameterToOptimize(String name, double min, double max, ListOfParametersToOptimize listOfParametersToOptimize)
   {
      this(name, min, max, 16, listOfParametersToOptimize);
   }

   public DoubleParameterToOptimize(String name, double min, double max, int bitsOfResolution, ListOfParametersToOptimize listOfParametersToOptimize)
   {
      super(name);

      this.min = min;
      this.max = max;

      if (bitsOfResolution < 1)
         throw new RuntimeException("bitsOfResolution < 1");

      this.bitsOfResolution = bitsOfResolution;

      listOfParametersToOptimize.addParameterToOptimize(this);
   }

   public int getBitsOfResolution()
   {
      return bitsOfResolution;
   }

   public double getValueGivenZeroToOne(double zeroToOne)
   {
      return min + zeroToOne * (max - min);
   }

   public double getZeroToOneGivenValue(double value)
   {
      if (max == min)
         return 0.0;

      return (value - min) / (max - min);
   }

   public ParameterToOptimizeType getType()
   {
      return ParameterToOptimizeType.DOUBLE;
   }

   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      currentValue = getValueGivenZeroToOne(zeroToOne);
   }

   public double getCurrentValueFromZeroToOne()
   {
      return getZeroToOneGivenValue(currentValue);
   }

   public double getCurrentValue()
   {
      return currentValue;
   }

   public void setCurrentValue(ParameterToOptimize parameterToOptimize)
   {
      double newValue = ((DoubleParameterToOptimize) parameterToOptimize).getCurrentValue();
      setCurrentValue(newValue);
   }

   public void setCurrentValue(double newValue)
   {
      this.currentValue = newValue;
   }

   public String toString()
   {
      return this.getName() + ": " + currentValue + ", [" + min + ", " + max + "], (" + bitsOfResolution + ")";
   }

   public double getCurrentValueAsADouble()
   {
      return getCurrentValue();
   }
}

package us.ihmc.utilities.parameterOptimization;

public class IntegerParameterToOptimize extends ParameterToOptimize
{
   private final int min, max;
   private final int bitsOfResolution;
   private int currentValue;
   
   public IntegerParameterToOptimize(String name, int min, int max, ListOfParametersToOptimize listOfParametersToOptimize)
   {
      super(name);
      
      this.min = min;
      this.max = max;
      
      this.bitsOfResolution = Math.max(1, (int) (Math.ceil(Math.log(1.0 + Math.abs(max - min)) / Math.log(2.0))));
      
      listOfParametersToOptimize.addParameterToOptimize(this);
   }
   
   public int getBitsOfResolution()
   {
      return bitsOfResolution;
   }
   
   public int getValueGivenZeroToOne(double zeroToOne)
   {
      return (int) (Math.round(((double) min) + zeroToOne * ((double) (max - min))));
   }
   
   public double getZeroToOneGivenValue(int value)
   {
      if (max == min) return 0.0;
      
      return ((double) (value - min))/((double) (max - min));
   }

   public ParameterToOptimizeType getType()
   {
      return ParameterToOptimizeType.INTEGER;
   }

   public void setCurrentValueGivenZeroToOne(double zeroToOne)
   {
      currentValue = getValueGivenZeroToOne(zeroToOne);
   }
   
   public double getCurrentValueFromZeroToOne()
   {
      return getZeroToOneGivenValue(currentValue);
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
   
   public String toString()
   {
      return this.getName() + ": " + currentValue + ", [" + min + ", " + max + "], (" + bitsOfResolution + ")";
   }

   public double getCurrentValueAsADouble()
   {
      return (double) getCurrentValue();
   }
   
}

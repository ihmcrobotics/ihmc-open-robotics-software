package us.ihmc.util.parameterOptimization;

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

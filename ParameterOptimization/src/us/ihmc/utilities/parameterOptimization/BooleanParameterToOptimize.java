package us.ihmc.utilities.parameterOptimization;


public class BooleanParameterToOptimize extends ParameterToOptimize
{   
   private boolean currentValue;
   
   public BooleanParameterToOptimize(String name, ListOfParametersToOptimize listOfParametersToOptimize)
   {
      super(name);
      listOfParametersToOptimize.addParameterToOptimize(this);
   }
   
   public int getBitsOfResolution()
   {
      return 1;
   }
   
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
   
   public double getCurrentValueFromZeroToOne()
   {
      return getCurrentValueAsADouble();
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
   
   public String toString()
   {
      return this.getName() + ": " + currentValue;
   }

   public double getCurrentValueAsADouble()
   {
      if (currentValue) return 1.0;
      else return 0.0;
   }
}

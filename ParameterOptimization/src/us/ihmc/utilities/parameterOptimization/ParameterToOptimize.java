package us.ihmc.utilities.parameterOptimization;

public abstract class ParameterToOptimize
{
   private final String name;
   
   public ParameterToOptimize(String name)
   {
      this.name = name;
   }
   
   public String getName()
   {
      return name;
   }
   
   public abstract ParameterToOptimizeType getType();
   public abstract void setCurrentValueGivenZeroToOne(double zeroToOne);
   public abstract double getCurrentValueFromZeroToOne();

   public abstract void setCurrentValue(ParameterToOptimize parameterToOptimize);
   public abstract int getBitsOfResolution();

   public abstract double getCurrentValueAsADouble();         
}

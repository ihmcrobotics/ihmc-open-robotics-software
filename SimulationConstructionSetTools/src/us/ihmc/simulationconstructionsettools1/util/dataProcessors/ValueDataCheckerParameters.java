package us.ihmc.simulationconstructionsettools1.util.dataProcessors;

public class ValueDataCheckerParameters
{
   private double maximumDerivative = Double.POSITIVE_INFINITY;
   private double maximumSecondDerivative = Double.POSITIVE_INFINITY;
   private double maximumValue = Double.POSITIVE_INFINITY;
   private double minimumValue = Double.NEGATIVE_INFINITY;
   private double errorThresholdOnDerivativeComparison = Double.POSITIVE_INFINITY;
   
   public ValueDataCheckerParameters()
   {
      
   }
 
   public ValueDataCheckerParameters getDefensiveCopy()
   {
      ValueDataCheckerParameters ret = new ValueDataCheckerParameters();
      ret.setErrorThresholdOnDerivativeComparison(this.getErrorThresholdOnDerivativeComparison());
      ret.setMaximumDerivative(this.getMaximumDerivative());
      ret.setMaximumSecondDerivative(this.getMaximumSecondDerivative());
      ret.setMaximumValue(this.getMaximumValue());
      ret.setMinimumValue(this.getMinimumValue());
      
      return ret;
   }
 
   public double getErrorThresholdOnDerivativeComparison()
   {
      return errorThresholdOnDerivativeComparison;
   }

   public void setErrorThresholdOnDerivativeComparison(double errorThresholdOnDerivativeComparison)
   {
      this.errorThresholdOnDerivativeComparison = Math.abs(errorThresholdOnDerivativeComparison);
   }

 
   public double getMaximumDerivative()
   {
      return maximumDerivative;
   }

   public void setMaximumDerivative(double maximumDerivative)
   {
      this.maximumDerivative = Math.abs(maximumDerivative);
   }

   public double getMaximumSecondDerivative()
   {
      return maximumSecondDerivative;
   }

   public void setMaximumSecondDerivative(double maximumSecondDerivative)
   {
      this.maximumSecondDerivative = Math.abs(maximumSecondDerivative);
   }

   public double getMaximumValue()
   {
      return maximumValue;
   }

   public void setMaximumValue(double maximumValue)
   {
      if (maximumValue < minimumValue)
         throw new RuntimeException("maximumValue must be greater than minimumValue. maximumValue=" + maximumValue + ", minimumValue=" + minimumValue);
      
      this.maximumValue = maximumValue;
   }

   public double getMinimumValue()
   {
      return minimumValue;
   }

   public void setMinimumValue(double minimumValue)
   {
      if (minimumValue > maximumValue)
         throw new RuntimeException("maximumValue must be greater than minimumValue. maximumValue=" + maximumValue + ", minimumValue=" + minimumValue);
      
      this.minimumValue = minimumValue;
   }
}

package us.ihmc.kalman.comparisons;

public interface ExampleFunctionController
{

   public abstract double getValue();

   public abstract double getFirstDerivative();

   public abstract double getSecondDerivative();
   
   public abstract boolean isPositionMeasurementUpdated();

}
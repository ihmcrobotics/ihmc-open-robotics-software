package us.ihmc.simulationconstructionset.util.dataProcessors;

import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class YoVariableValueDataChecker implements DataProcessingFunction
{
   private BooleanYoVariable maximumValueExceeded;
   private BooleanYoVariable minimumValueExceeded;
   private BooleanYoVariable maximumDerivativeExceeded;
   private BooleanYoVariable maximumSecondDerivativeExceeded;
   
   private DoubleYoVariable calculatedDerivative;
   private boolean hasBeenInitalized = false;
   private double previousValue;
   private double previousTime;
   private double previousDerivative = Double.NaN;

   private DoubleYoVariable time;
   private DoubleYoVariable variableToCheck;

   private double maximumDerivative = Double.POSITIVE_INFINITY;
   private double maximumSecondDerivative = Double.POSITIVE_INFINITY;
   private double maximumValue = Double.POSITIVE_INFINITY;
   private double minimumValue = Double.NEGATIVE_INFINITY;

   private boolean maxDerivativeExeeded = false;
   

   private boolean maxSecondDerivativeExeeded = false;
   private boolean maxValueExeeded = false;
   private boolean minValueExeeded = false;
   private DoubleYoVariable calculatedSecondDerivative;

   
   public YoVariableValueDataChecker(SimulationConstructionSet scs, DoubleYoVariable variableToCheck, DoubleYoVariable time)
   {
      YoVariableRegistry registry = scs.getRootRegistry();

      this.time = time;
      this.variableToCheck = variableToCheck;

      maximumValueExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MaxValueExceeded", registry);
      minimumValueExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MinValueExceeded", registry);
      maximumDerivativeExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MaxDervExceeded", registry);
      maximumSecondDerivativeExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MaxSecDervExceeded", registry);
      
      calculatedDerivative = new DoubleYoVariable(variableToCheck.getName() + "_CalcDerv", registry);
      calculatedSecondDerivative = new DoubleYoVariable(variableToCheck.getName() + "_CalcSecDerv", registry);

      hasBeenInitalized = false;
   }

   public void setMaximumDerivative(double maximumDerivative)
   {
      this.maximumDerivative = Math.abs( maximumDerivative );
   }
   public void setMaximumSecondDerivate(double maximumSecondDerivative)
   {
      this.maximumSecondDerivative = Math.abs(maximumSecondDerivative);
   }
   
   public void setMaximumValue(double maximumValue)
   {
      if (maximumValue < minimumValue)
         throw new RuntimeException("maximumValue must be greater than maximumValue. maximumValue=" + maximumValue + ", maximumValue=" + maximumValue);

      this.maximumValue = maximumValue ;
   }
   
   public void setMinimumValue(double minimumValue)
   {
      if (minimumValue > maximumValue)
         throw new RuntimeException("maximumValue must be greater than maximumValue. maximumValue=" + maximumValue + ", maximumValue=" + maximumValue);
      
      this.minimumValue = minimumValue;
   }

   @Override
   public void initializeProcessing()
   {
      hasBeenInitalized = false;
      maxDerivativeExeeded = false;
      maxSecondDerivativeExeeded = false;
      maxValueExeeded = false;
      minValueExeeded = false;
      previousDerivative = Double.NaN;
   }

   @Override
   public void processData()
   {
      if (hasBeenInitalized)
      {
//       calculatedDerivative =  (variableToCheck.getvalue - variableToCheck.getpreviousvalue)/deltaT
//       if abs(calculatedDerivative ) > max set true

         double currentValue = variableToCheck.getDoubleValue();
         double currentTime = time.getDoubleValue();
         double currentDerivative = (currentValue - previousValue) / (currentTime - previousTime);
        
         double currentSecondDerivative = 0.0;
         
         if( Double.isNaN(previousDerivative) == false )
           currentSecondDerivative = (currentDerivative - previousDerivative) / (currentTime - previousTime);

         minimumValueExceeded.set( currentValue < minimumValue );
         maximumValueExceeded.set( currentValue > maximumValue );
     
         calculatedDerivative.set(currentDerivative);
         calculatedSecondDerivative.set(currentSecondDerivative);

         maximumDerivativeExceeded.set(       Math.abs(currentDerivative)       > maximumDerivative );
         maximumSecondDerivativeExceeded.set( Math.abs(currentSecondDerivative) > maximumSecondDerivative );

         maxValueExeeded = maxValueExeeded || maximumValueExceeded.getBooleanValue();
         minValueExeeded = minValueExeeded || minimumValueExceeded.getBooleanValue();
         
         maxDerivativeExeeded       = maxDerivativeExeeded       || maximumDerivativeExceeded.getBooleanValue();
         maxSecondDerivativeExeeded = maxSecondDerivativeExeeded || maximumSecondDerivativeExceeded.getBooleanValue();
         
         previousTime = currentTime;
         previousValue = currentValue;
         previousDerivative = currentDerivative;
      }
      else
      {
         previousTime = time.getDoubleValue();
         previousValue = variableToCheck.getDoubleValue();
         previousDerivative = 0.0;
         hasBeenInitalized = true;
      }
   }
   
   public boolean isMaxDerivativeExeeded()
   {
      return maxDerivativeExeeded;
   }

   public boolean isMaxSecondDerivativeExeeded()
   {
      return maxSecondDerivativeExeeded;
   }

   public boolean isMaxValueExeeded()
   {
      return maxValueExeeded;
   }

   public boolean isMinValueExeeded()
   {
      return minValueExeeded;
   }

}

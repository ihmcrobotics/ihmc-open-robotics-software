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
   private BooleanYoVariable derivativeCompError;


   private DoubleYoVariable calculatedDerivative;
   private double previousValue;
   private double previousTime;
   private double previousDerivative = Double.NaN;

   private DoubleYoVariable time;
   private DoubleYoVariable variableToCheck;
   private DoubleYoVariable actualDerivativeofVariableToCheck;

   private boolean maxDerivativeExeeded = false;

   private boolean maxSecondDerivativeExeeded = false;
   private boolean maxValueExeeded = false;
   private boolean minValueExeeded = false;
   private boolean derivativeCompErrorOccurred = false;

   private DoubleYoVariable calculatedSecondDerivative;

   private int counter;
   
   private SimulationConstructionSet scs;

   private ValueDataCheckerParameters valueDataCheckerParameters;


   public YoVariableValueDataChecker(SimulationConstructionSet scs, DoubleYoVariable variableToCheck, DoubleYoVariable time,
                                     ValueDataCheckerParameters valueDataCheckerParameters)
   {
      this(scs, variableToCheck, time, valueDataCheckerParameters, null);
   }


   public YoVariableValueDataChecker(SimulationConstructionSet scs, DoubleYoVariable variableToCheck, DoubleYoVariable time,
                                     ValueDataCheckerParameters valueDataCheckerParameters, DoubleYoVariable actualDerivativeOfVariableToCheck)
   {
      this.scs = scs;
      
      YoVariableRegistry registry = scs.getRootRegistry();

      this.time = time;
      this.variableToCheck = variableToCheck;

      this.valueDataCheckerParameters = valueDataCheckerParameters.getDefensiveCopy();

      this.actualDerivativeofVariableToCheck = actualDerivativeOfVariableToCheck;

      maximumValueExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MaxValueExceeded", registry);
      minimumValueExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MinValueExceeded", registry);
      maximumDerivativeExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MaxDervExceeded", registry);
      maximumSecondDerivativeExceeded = new BooleanYoVariable(variableToCheck.getName() + "_MaxSecDervExceeded", registry);

      if (actualDerivativeofVariableToCheck != null)
         derivativeCompError = new BooleanYoVariable(variableToCheck.getName() + "_DerivativeCompError", registry);

      calculatedDerivative = new DoubleYoVariable(variableToCheck.getName() + "_CalcDerv", registry);
      calculatedSecondDerivative = new DoubleYoVariable(variableToCheck.getName() + "_CalcSecDerv", registry);

      counter = 0;
   }
   
   public void cropFirstPoint()
   {
      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   public void setMaximumDerivative(double maximumDerivative)
   {
      valueDataCheckerParameters.setMaximumDerivative(maximumDerivative);
   }

   public void setMaximumSecondDerivate(double maximumSecondDerivative)
   {
      valueDataCheckerParameters.setMaximumSecondDerivative(maximumSecondDerivative);
   }

   public void setMaximumValue(double maximumValue)
   {
      valueDataCheckerParameters.setMaximumValue(maximumValue);
   }

   public void setMinimumValue(double minimumValue)
   {
      valueDataCheckerParameters.setMinimumValue(minimumValue);
   }
   
   public void setErrorThresholdOnDerivativeComparison(double errorThresholdOnDerivativeComparison)
   {
      valueDataCheckerParameters.setErrorThresholdOnDerivativeComparison(errorThresholdOnDerivativeComparison);
   }
   
   public void setValueDataCheckerParameters(ValueDataCheckerParameters valueDataCheckerParameters)
   {
      this.valueDataCheckerParameters = valueDataCheckerParameters.getDefensiveCopy();
   }
   
   public ValueDataCheckerParameters getValueDataCheckerParametersCopy()
   {
      return valueDataCheckerParameters.getDefensiveCopy();
   }

   @Override
   public void initializeProcessing()
   {
      maxDerivativeExeeded = false;
      maxSecondDerivativeExeeded = false;
      maxValueExeeded = false;
      minValueExeeded = false;
      previousDerivative = Double.NaN;
      derivativeCompErrorOccurred = false;
      counter = 0;
   }

   @Override
   public void processData()
   {
      double currentValue = variableToCheck.getDoubleValue();
      minimumValueExceeded.set(currentValue < valueDataCheckerParameters.getMinimumValue());
      maximumValueExceeded.set(currentValue > valueDataCheckerParameters.getMaximumValue());

      double currentTime = time.getDoubleValue();

      double currentDerivative;
      double currentSecondDerivative;

      if (counter >= 1)
      {
         currentDerivative = (currentValue - previousValue) / (currentTime - previousTime);

         if (actualDerivativeofVariableToCheck != null)
         {
            if (Math.abs(actualDerivativeofVariableToCheck.getDoubleValue() - currentDerivative) > valueDataCheckerParameters.getErrorThresholdOnDerivativeComparison())
            {
               derivativeCompError.set(true);
               derivativeCompErrorOccurred = true;
            }
            else
               derivativeCompError.set(false);
         }
      }
      else
      {
         currentDerivative = 0.0;
      }

      if (counter >= 2)
      {
         currentSecondDerivative = (currentDerivative - previousDerivative) / (currentTime - previousTime);
      }
      else
      {
         currentSecondDerivative = 0.0;
      }


      calculatedDerivative.set(currentDerivative);
      calculatedSecondDerivative.set(currentSecondDerivative);

      maximumDerivativeExceeded.set(Math.abs(currentDerivative) >  valueDataCheckerParameters.getMaximumDerivative());
      maximumSecondDerivativeExceeded.set(Math.abs(currentSecondDerivative) > valueDataCheckerParameters.getMaximumSecondDerivative());

      maxValueExeeded = maxValueExeeded || maximumValueExceeded.getBooleanValue();
      minValueExeeded = minValueExeeded || minimumValueExceeded.getBooleanValue();

      maxDerivativeExeeded = maxDerivativeExeeded || maximumDerivativeExceeded.getBooleanValue();
      maxSecondDerivativeExeeded = maxSecondDerivativeExeeded || maximumSecondDerivativeExceeded.getBooleanValue();

      previousTime = currentTime;
      previousValue = currentValue;
      previousDerivative = currentDerivative;

      counter++;
   }

   public boolean isDerivativeCompErrorOccurred()
   {
      return derivativeCompErrorOccurred;
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

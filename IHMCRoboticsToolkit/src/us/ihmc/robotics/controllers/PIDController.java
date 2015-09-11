package us.ihmc.robotics.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class PIDController
{
   private final PDController pdController;
   private final DoubleYoVariable integralGain;
   private final DoubleYoVariable maxIntegralError;
   private final DoubleYoVariable maxOutput;
   private final DoubleYoVariable cumulativeError;
   private final DoubleYoVariable integralLeakRatio;
   
   private final DoubleYoVariable actionI;

   public PIDController(String suffix, YoVariableRegistry registry)
   {
      pdController = new PDController(suffix, registry);

      integralGain = new DoubleYoVariable("ki_" + suffix, registry);
      integralGain.set(0.0);

      maxIntegralError = new DoubleYoVariable("maxIntegralError_" + suffix, registry);
      maxIntegralError.set(Double.POSITIVE_INFINITY);
      
      maxOutput = new DoubleYoVariable("maxOutput_" + suffix, registry);
      maxOutput.set(Double.POSITIVE_INFINITY);

      cumulativeError = new DoubleYoVariable("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);
      
      actionI = new DoubleYoVariable("action_I_" + suffix, registry);
      actionI.set(0.0);
      
      integralLeakRatio = new DoubleYoVariable("leak_" + suffix, registry);     
      integralLeakRatio.set(1.0);
      
   }

   public PIDController(DoubleYoVariable proportionalGain, DoubleYoVariable integralGain, DoubleYoVariable derivativeGain, DoubleYoVariable maxIntegralError,
         String suffix, YoVariableRegistry registry)
   {
      pdController = new PDController(proportionalGain, derivativeGain, suffix, registry);
      this.integralGain = integralGain;
      this.maxIntegralError = maxIntegralError;
      
      maxOutput = new DoubleYoVariable("maxOutput_" + suffix, registry);
      maxOutput.set(Double.POSITIVE_INFINITY);

      cumulativeError = new DoubleYoVariable("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);
      
      actionI = new DoubleYoVariable("integralAction_" + suffix, registry);
      actionI.set(0.0);
      
      integralLeakRatio = new DoubleYoVariable("leak_" + suffix, registry);     
      integralLeakRatio.set(1.0);
   }
   
   public double getMaximumOutputLimit()
   {
      return maxOutput.getDoubleValue();
   }

   public void setMaximumOutputLimit(double max)
   {
      if( max <=0 )
         maxOutput.set(Double.POSITIVE_INFINITY);
      else
         maxOutput.set( max );
   }

   public double getProportionalGain()
   {
      return pdController.getProportionalGain();
   }

   public double getDerivativeGain()
   {
      return pdController.getDerivativeGain();
   }

   public void setProportionalGain(double proportionalGain)
   {
      pdController.setProportionalGain(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      pdController.setDerivativeGain(derivativeGain);
   }

   public double getPositionError()
   {
      return pdController.getPositionError();
   }

   public double getRateError()
   {
      return pdController.getRateError();
   }

   public double getCumulativeError()
   {
      return cumulativeError.getDoubleValue();
   }

   public void setCumulativeError(double error)
   {
      cumulativeError.set(error);
   }

   public double getIntegralGain()
   {
      return integralGain.getDoubleValue();
   }

   public double getMaxIntegralError()
   {
      return maxIntegralError.getDoubleValue();
   }

   public void setIntegralGain(double integralGain)
   {
      this.integralGain.set(integralGain);
   }
   
   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio.set(integralLeakRatio);
   }
   
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio.getDoubleValue();
   }

   public void setMaxIntegralError(double maxIntegralError)
   {
      this.maxIntegralError.set(maxIntegralError);
   }

   public void resetIntegrator()
   {
      cumulativeError.set(0.0);
   }

   public double compute(double currentPosition, double desiredPosition, double currentRate, double desiredRate, double deltaTime)
   {
      pdController.compute(currentPosition, desiredPosition, currentRate, desiredRate);

      return computeIntegralEffortAndAddPDEffort(deltaTime);
   }

   public double computeForAngles(double currentPosition, double desiredPosition, double currentRate, double desiredRate, double deltaTime)
   {
      pdController.computeForAngles(currentPosition, desiredPosition, currentRate, desiredRate);

      return computeIntegralEffortAndAddPDEffort(deltaTime);
   }

   private double computeIntegralEffortAndAddPDEffort(double deltaTime)
   {
      // LIMIT THE MAX INTEGRAL ERROR SO WON'T WIND UP
      double maxError = maxIntegralError.getDoubleValue();
      cumulativeError.set(pdController.getPositionError() * deltaTime + integralLeakRatio.getDoubleValue()*cumulativeError.getDoubleValue());
      if (cumulativeError.getDoubleValue() > maxError)
         cumulativeError.set(maxError);
      else if (cumulativeError.getDoubleValue() < -maxError)
         cumulativeError.set(-maxError);
      
      actionI.set(integralGain.getDoubleValue() * cumulativeError.getDoubleValue());

      double outputSignal = (pdController.getProportionalGain() * pdController.getPositionError()) + (integralGain.getDoubleValue() * cumulativeError.getDoubleValue())
            + (pdController.getDerivativeGain() * pdController.getRateError());
      
      double maximumOutput = Math.abs( maxOutput.getDoubleValue() );
      
      if( outputSignal >  maximumOutput) return  maximumOutput;
      if( outputSignal < -maximumOutput) return -maximumOutput;
      return outputSignal;
   }
}

package us.ihmc.robotics.controllers;

import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class PIDController
{
   private final PDController pdController;
   private final YoDouble integralGain;
   private final YoDouble maxIntegralError;
   private final YoDouble maxOutput;
   private final YoDouble cumulativeError;
   private final YoDouble integralLeakRatio;
   
   private final YoDouble actionI;

   public PIDController(String suffix, YoVariableRegistry registry)
   {
      pdController = new PDController(suffix, registry);

      integralGain = new YoDouble("ki_" + suffix, registry);
      integralGain.set(0.0);

      maxIntegralError = new YoDouble("maxIntegralError_" + suffix, registry);
      maxIntegralError.set(Double.POSITIVE_INFINITY);
      
      maxOutput = new YoDouble("maxOutput_" + suffix, registry);
      maxOutput.set(Double.POSITIVE_INFINITY);

      cumulativeError = new YoDouble("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);
      
      actionI = new YoDouble("action_I_" + suffix, registry);
      actionI.set(0.0);
      
      integralLeakRatio = new YoDouble("leak_" + suffix, registry);
      integralLeakRatio.set(1.0);

      VariableChangedListener leakRatioClipper = new VariableChangedListener()
      {
         @Override public void notifyOfVariableChange(YoVariable<?> v)
         {
            integralLeakRatio.set(MathTools.clamp(integralLeakRatio.getDoubleValue(), 0.0, 1.0), false);
         }
      };

      integralLeakRatio.addVariableChangedListener(leakRatioClipper);
   }

   public PIDController(YoDouble proportionalGain, YoDouble integralGain, YoDouble derivativeGain, YoDouble maxIntegralError,
         String suffix, YoVariableRegistry registry)
   {
      pdController = new PDController(proportionalGain, derivativeGain, suffix, registry);
      this.integralGain = integralGain;
      this.maxIntegralError = maxIntegralError;
      
      maxOutput = new YoDouble("maxOutput_" + suffix, registry);
      maxOutput.set(Double.POSITIVE_INFINITY);

      cumulativeError = new YoDouble("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);
      
      actionI = new YoDouble("integralAction_" + suffix, registry);
      actionI.set(0.0);
      
      integralLeakRatio = new YoDouble("leak_" + suffix, registry);
      integralLeakRatio.set(1.0);
   }

   public PIDController(YoPIDGains yoPIDGains, String suffix, YoVariableRegistry registry)
   {
      pdController = new PDController(yoPIDGains, suffix, registry);
      this.integralGain = yoPIDGains.getYoKi();
      this.maxIntegralError = yoPIDGains.getYoMaxIntegralError();
      this.maxOutput = yoPIDGains.getYoMaximumOutput();

      cumulativeError = new YoDouble("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);

      actionI = new YoDouble("integralAction_" + suffix, registry);
      actionI.set(0.0);

      integralLeakRatio = yoPIDGains.getYoIntegralLeakRatio();
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

   public void setPositionDeadband(double deadband)
   {
      pdController.setPositionDeadband(deadband);
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

   public double getPositionDeadband()
   {
      return pdController.getPositionDeadband();
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
      this.integralLeakRatio.set(MathTools.clamp(integralLeakRatio, 0.0, 1.0));
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
      double outputSignal = (pdController.getProportionalGain() * pdController.getPositionError()) + (pdController.getDerivativeGain() * pdController.getRateError());

      if (integralGain.getDoubleValue() < 1.0e-5)
      {
         cumulativeError.set(0.0);
      }
      else
      {
         // LIMIT THE MAX INTEGRAL ERROR SO WON'T WIND UP
         double maxError = maxIntegralError.getDoubleValue();
         double errorAfterLeak = pdController.getPositionError() * deltaTime + integralLeakRatio.getDoubleValue() * cumulativeError.getDoubleValue();
         cumulativeError.set(MathTools.clamp(errorAfterLeak, maxError));

         actionI.set(integralGain.getDoubleValue() * cumulativeError.getDoubleValue());
         outputSignal += actionI.getDoubleValue();
      }

      double maximumOutput = Math.abs( maxOutput.getDoubleValue() );
      outputSignal = MathTools.clamp(outputSignal, maximumOutput);
      return outputSignal;
   }
}

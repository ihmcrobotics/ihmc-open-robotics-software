package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterizedPIDController extends AbstractPIDController
{
   final ParameterizedPDController pdController;
   final DoubleProvider integralGain;
   final DoubleProvider maxIntegralError;
   final DoubleProvider maxOutput;
   final DoubleProvider integralLeakRatio;

   public ParameterizedPIDController(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);

      this.pdController = new ParameterizedPDController(suffix, registry);
      integralGain = new DoubleParameter("ki_" + suffix, registry, 0.0);

      maxIntegralError = new DoubleParameter("maxIntegralError_" + suffix, registry, Double.POSITIVE_INFINITY);

      maxOutput = new DoubleParameter("maxOutput_" + suffix, registry, Double.POSITIVE_INFINITY);

      integralLeakRatio = new DoubleParameter("leak_" + suffix, registry, 1.0);

   }

   public ParameterizedPIDController(DoubleParameter proportionalGain, DoubleParameter integralGain, DoubleParameter derivativeGain,
                                     DoubleParameter maxIntegralError, String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      this.pdController = new ParameterizedPDController(proportionalGain, derivativeGain, suffix, registry);
      this.integralGain = integralGain;
      this.maxIntegralError = maxIntegralError;
      this.maxOutput = () -> Double.POSITIVE_INFINITY;
      this.integralLeakRatio = () -> 1.0;
   }

   @Override
   protected AbstractPDController getPDController()
   {
      return pdController;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maxOutput.getValue();
   }

   @Override
   public double getIntegralGain()
   {
      return integralGain.getValue();
   }

   @Override
   public double getMaxIntegralError()
   {
      return maxIntegralError.getValue();
   }

   @Override
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio.getValue();
   }

}

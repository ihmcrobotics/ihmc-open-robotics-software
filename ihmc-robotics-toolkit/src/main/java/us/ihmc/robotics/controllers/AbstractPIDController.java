package us.ihmc.robotics.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AbstractPIDController extends AbstractPDController
{
   private final YoDouble cumulativeError;
   private final YoDouble actionI;

   protected final DoubleProvider integralGain;
   protected final DoubleProvider maxIntegralError;
   protected final DoubleProvider maxFeedback;
   protected final DoubleProvider integralLeakRatio;

   protected AbstractPIDController(DoubleProvider proportionalGain,
                                   DoubleProvider integralGain,
                                   DoubleProvider derivativeGain,
                                   DoubleProvider positionDeadband,
                                   DoubleProvider maxIntegralError,
                                   DoubleProvider maxFeedback,
                                   DoubleProvider integralLeakRatio,
                                   String suffix, YoRegistry registry)
   {
      super(proportionalGain, derivativeGain, positionDeadband, suffix, registry);

      this.integralGain = integralGain;
      this.maxIntegralError = maxIntegralError;
      this.maxFeedback = maxFeedback;
      this.integralLeakRatio = integralLeakRatio;

      cumulativeError = new YoDouble("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);

      actionI = new YoDouble("action_I_" + suffix, registry);
      actionI.set(0.0);
   }

   public double getMaximumFeedback()
   {
      return maxFeedback.getValue();
   }

   public double getIntegralGain()
   {
      return integralGain.getValue();
   }

   public double getMaxIntegralError()
   {
      return maxIntegralError.getValue();
   }

   public double getIntegralLeakRatio()
   {
      return integralLeakRatio.getValue();
   }

   public double getCumulativeError()
   {
      return cumulativeError.getDoubleValue();
   }

   public void setCumulativeError(double error)
   {
      cumulativeError.set(error);
   }

   public void resetIntegrator()
   {
      cumulativeError.set(0.0);
   }

   public double compute(double currentPosition, double desiredPosition, double currentRate, double desiredRate, double deltaTime)
   {
      super.compute(currentPosition, desiredPosition, currentRate, desiredRate);

      return computeIntegralEffortAndAddPDEffort(deltaTime);
   }

   public double computeForAngles(double currentPosition, double desiredPosition, double currentRate, double desiredRate, double deltaTime)
   {
      super.computeForAngles(currentPosition, desiredPosition, currentRate, desiredRate);

      return computeIntegralEffortAndAddPDEffort(deltaTime);
   }

   private double computeIntegralEffortAndAddPDEffort(double deltaTime)
   {
      double outputSignal = (actionP.getDoubleValue() + actionD.getDoubleValue());

      if (integralGain.getValue() < 1.0e-5)
      {
         cumulativeError.set(0.0);
      }
      else
      {
         // LIMIT THE MAX INTEGRAL ERROR SO WON'T WIND UP
         double maxError = maxIntegralError.getValue();
         double errorAfterLeak = positionError.getDoubleValue() * deltaTime + integralLeakRatio.getValue() * cumulativeError.getDoubleValue();
         cumulativeError.set(MathTools.clamp(errorAfterLeak, maxError));

         actionI.set(integralGain.getValue() * cumulativeError.getDoubleValue());
         outputSignal += actionI.getDoubleValue();
      }

      double maximumOutput = Math.abs(maxFeedback.getValue());
      outputSignal = MathTools.clamp(outputSignal, maximumOutput);
      return outputSignal;
   }

}

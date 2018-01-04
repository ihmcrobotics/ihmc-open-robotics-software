package us.ihmc.robotics.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AbstractPIDController
{
   private final YoDouble cumulativeError;
   private final YoDouble actionI;

   protected AbstractPIDController(String suffix, YoVariableRegistry registry)
   {
      cumulativeError = new YoDouble("cumulativeError_" + suffix, registry);
      cumulativeError.set(0.0);

      actionI = new YoDouble("action_I_" + suffix, registry);
      actionI.set(0.0);
   }

   protected abstract AbstractPDController getPDController();

   public abstract double getMaximumFeedback();

   public abstract double getIntegralGain();

   public abstract double getMaxIntegralError();

   public abstract double getIntegralLeakRatio();

   public double getProportionalGain()
   {
      return getPDController().getProportionalGain();
   }

   public double getDerivativeGain()
   {
      return getPDController().getDerivativeGain();
   }

   public double getPositionError()
   {
      return getPDController().getPositionError();
   }

   public double getRateError()
   {
      return getPDController().getRateError();
   }

   public double getCumulativeError()
   {
      return cumulativeError.getDoubleValue();
   }

   public void setCumulativeError(double error)
   {
      cumulativeError.set(error);
   }

   public double getPositionDeadband()
   {
      return getPDController().getPositionDeadband();
   }

   public void resetIntegrator()
   {
      cumulativeError.set(0.0);
   }

   public double compute(double currentPosition, double desiredPosition, double currentRate, double desiredRate, double deltaTime)
   {
      getPDController().compute(currentPosition, desiredPosition, currentRate, desiredRate);

      return computeIntegralEffortAndAddPDEffort(deltaTime);
   }

   public double computeForAngles(double currentPosition, double desiredPosition, double currentRate, double desiredRate, double deltaTime)
   {
      getPDController().computeForAngles(currentPosition, desiredPosition, currentRate, desiredRate);

      return computeIntegralEffortAndAddPDEffort(deltaTime);
   }

   private double computeIntegralEffortAndAddPDEffort(double deltaTime)
   {
      double outputSignal = (getPDController().getProportionalGain() * getPDController().getPositionError())
            + (getPDController().getDerivativeGain() * getPDController().getRateError());

      if (getIntegralGain() < 1.0e-5)
      {
         cumulativeError.set(0.0);
      }
      else
      {
         // LIMIT THE MAX INTEGRAL ERROR SO WON'T WIND UP
         double maxError = getMaxIntegralError();
         double errorAfterLeak = getPDController().getPositionError() * deltaTime + getIntegralLeakRatio() * cumulativeError.getDoubleValue();
         cumulativeError.set(MathTools.clamp(errorAfterLeak, maxError));

         actionI.set(getIntegralGain() * cumulativeError.getDoubleValue());
         outputSignal += actionI.getDoubleValue();
      }

      double maximumOutput = Math.abs(getMaximumFeedback());
      outputSignal = MathTools.clamp(outputSignal, maximumOutput);
      return outputSignal;
   }

}

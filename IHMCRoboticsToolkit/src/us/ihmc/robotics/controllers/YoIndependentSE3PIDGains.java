package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoIndependentSE3PIDGains implements YoSE3PIDGainsInterface
{
   private final YoEuclideanPositionGains positionGains;
   private final YoAxisAngleOrientationGains orientationGains;

   public YoIndependentSE3PIDGains(String prefix, YoVariableRegistry registry)
   {
      this(prefix, false, registry);
   }

   public YoIndependentSE3PIDGains(String prefix, boolean createDampingRatio, YoVariableRegistry registry)
   {
      positionGains = new YoEuclideanPositionGains(prefix, createDampingRatio, registry);
      orientationGains = new YoAxisAngleOrientationGains(prefix, createDampingRatio, registry);
   }

   public void reset()
   {
      positionGains.reset();
      orientationGains.reset();
   }

   @Override
   public YoEuclideanPositionGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public YoAxisAngleOrientationGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setPositionDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      positionGains.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setPositionIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      positionGains.setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   public void setPositionProportionalGains(double[] proportionalGains)
   {
      positionGains.setProportionalGains(proportionalGains);
   }

   public void setPositionDerivativeGains(double[] derivativeGains)
   {
      positionGains.setDerivativeGains(derivativeGains);
   }

   public void setPositionIntegralGains(double[] integralGains, double maxIntegralError)
   {
      positionGains.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setPositionDampingRatio(double dampingRatio)
   {
      positionGains.setDampingRatio(dampingRatio);
   }

   public void setPositionDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      positionGains.setDampingRatios(dampingRatioX, dampingRatioY, dampingRatioZ);
   }

   public void setPositionMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      positionGains.setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   public void setPositionMaxDerivativeError(double maxDerivativeError)
   {
      positionGains.setMaxDerivativeError(maxDerivativeError);
   }

   public void setPositionMaxProportionalError(double maxProportionalError)
   {
      positionGains.setMaxProportionalError(maxProportionalError);
   }

   public void setOrientationProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      orientationGains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setOrientationDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      orientationGains.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setOrientationIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      orientationGains.setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   public void setOrientationProportionalGains(double[] proportionalGains)
   {
      orientationGains.setProportionalGains(proportionalGains);
   }

   public void setOrientationDerivativeGains(double[] derivativeGains)
   {
      orientationGains.setDerivativeGains(derivativeGains);
   }

   public void setOrientationIntegralGains(double[] integralGains, double maxIntegralError)
   {
      orientationGains.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setOrientationDampingRatio(double dampingRatio)
   {
      orientationGains.setDampingRatio(dampingRatio);
   }

   public void setOrientationDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      orientationGains.setDampingRatios(dampingRatioX, dampingRatioY, dampingRatioZ);
   }

   public void setOrientationMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      orientationGains.setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   public void setOrientationMaxDerivativeError(double maxDerivativeError)
   {
      orientationGains.setMaxDerivativeError(maxDerivativeError);
   }

   public void setOrientationMaxProportionalError(double maxProportionalError)
   {
      orientationGains.setMaxProportionalError(maxProportionalError);
   }

   @Override
   public void set(SE3PIDGainsInterface gains)
   {
      set(gains.getPositionGains());
      set(gains.getOrientationGains());
   }

   @Override
   public void set(PositionPIDGainsInterface positionGains)
   {
      this.positionGains.set(positionGains);
   }

   @Override
   public void set(OrientationPIDGainsInterface orientationGains)
   {
      this.orientationGains.set(orientationGains);
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      positionGains.createDerivativeGainUpdater(updateNow);
      orientationGains.createDerivativeGainUpdater(updateNow);
   }
}

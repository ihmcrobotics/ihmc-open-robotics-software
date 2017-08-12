package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFootSE3Gains implements YoSE3PIDGainsInterface
{
   private final YoFootPositionGains positionGains;
   private final YoFootOrientationGains orientationGains;

   public YoFootSE3Gains(String prefix, YoVariableRegistry registry)
   {
      positionGains = new YoFootPositionGains(prefix, registry);
      orientationGains = new YoFootOrientationGains(prefix, registry);
   }

   @Override
   public YoFootPositionGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public YoFootOrientationGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionProportionalGains(double proportionalGainXY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainXY, proportionalGainZ);
   }

   public void setPositionProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setPositionDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      positionGains.setDerivativeGains(derivativeGainXY, derivativeGainZ);
   }

   public void setPositionDampingRatio(double dampingRatio)
   {
      positionGains.setDampingRatio(dampingRatio);
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

   public void setOrientationProportionalGains(double proportionalGainXY, double proportionalGainZ)
   {
      orientationGains.setProportionalGains(proportionalGainXY, proportionalGainZ);
   }

   public void setOrientationDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      orientationGains.setDerivativeGains(derivativeGainXY, derivativeGainZ);
   }

   public void setOrientationDampingRatio(double dampingRatio)
   {
      orientationGains.setDampingRatio(dampingRatio);
   }

   public void setOrientationDampingRatios(double dampingRatioXY, double dampingRatioZ)
   {
      orientationGains.setDampingRatios(dampingRatioXY, dampingRatioZ);
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
   public void setPositionGains(PID3DGainsReadOnly positionGains)
   {
      this.positionGains.set(positionGains);
   }

   @Override
   public void setOrientationGains(PID3DGainsReadOnly orientationGains)
   {
      this.orientationGains.set(orientationGains);
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      positionGains.createDerivativeGainUpdater(updateNow);
      orientationGains.createDerivativeGainUpdater(updateNow);
   }
}

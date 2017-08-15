package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.controllers.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFootSE3Gains implements YoPIDSE3Gains
{
   private final DefaultYoPID3DGains positionGains;
   private final DefaultYoPID3DGains orientationGains;

   public YoFootSE3Gains(String prefix, YoVariableRegistry registry)
   {
      positionGains = new DefaultYoPID3DGains(prefix + "Position", GainCoupling.XY, false, registry);
      orientationGains = new DefaultYoPID3DGains(prefix + "Orientation", GainCoupling.XY, false, registry);
   }

   @Override
   public YoPID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public YoPID3DGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionProportionalGains(double proportionalGainXY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainXY, proportionalGainXY, proportionalGainZ);
   }

   public void setPositionProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setPositionDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      positionGains.setDerivativeGains(derivativeGainXY, derivativeGainXY, derivativeGainZ);
   }

   public void setPositionDampingRatio(double dampingRatio)
   {
      positionGains.setDampingRatios(dampingRatio);
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
      orientationGains.setProportionalGains(proportionalGainXY, proportionalGainXY, proportionalGainZ);
   }

   public void setOrientationDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      orientationGains.setDerivativeGains(derivativeGainXY, derivativeGainXY, derivativeGainZ);
   }

   public void setOrientationDampingRatio(double dampingRatio)
   {
      orientationGains.setDampingRatios(dampingRatio);
   }

   public void setOrientationDampingRatios(double dampingRatioXY, double dampingRatioZ)
   {
      orientationGains.setDampingRatios(dampingRatioXY, dampingRatioXY, dampingRatioZ);
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
}

package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.controllers.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoPlanarFootSE3Gains implements YoPIDSE3Gains
{
   private final DefaultYoPID3DGains positionGains;
   private final DefaultYoPID3DGains orientationGains;

   public YoPlanarFootSE3Gains(String prefix, YoVariableRegistry registry)
   {
      positionGains = new DefaultYoPID3DGains(prefix + "Position", GainCoupling.XY, false, registry);
      orientationGains = new DefaultYoPID3DGains(prefix + "Orientation", GainCoupling.XYZ, false, registry);
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

   public void setPositionProportionalGains(double proportionalGains)
   {
      setPositionProportionalGains(proportionalGains, proportionalGains);
   }

   public void setPositionProportionalGains(double proportionalGainX, double proportionalGainY)
   {
      positionGains.setProportionalGains(proportionalGainX, proportionalGainX, proportionalGainY);
   }

   public void setPositionDerivativeGains(double derivativeGains)
   {
      setPositionDerivativeGains(derivativeGains, derivativeGains);
   }

   public void setPositionDerivativeGains(double derivativeGainX, double derivativeGainZ)
   {
      positionGains.setDerivativeGains(derivativeGainX, derivativeGainX, derivativeGainZ);
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

   public void setOrientationProportionalGains(double proportionalGainX)
   {
      orientationGains.setProportionalGains(proportionalGainX);
   }

   public void setOrientationDerivativeGains(double derivativeGainX)
   {
      orientationGains.setDerivativeGains(derivativeGainX);
   }

   public void setOrientationDampingRatio(double dampingRatio)
   {
      orientationGains.setDampingRatios(dampingRatio);
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

package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;


public class YoFootSE3Gains implements YoSE3PIDGains
{
   private final YoFootPositionGains positionGains;
   private final YoFootOrientationGains orientationGains;

   public YoFootSE3Gains(String prefix, YoVariableRegistry registry)
   {
      positionGains = new YoFootPositionGains(prefix, registry);
      orientationGains = new YoFootOrientationGains(prefix, registry);
   }

   public void reset()
   {
      positionGains.reset();
      orientationGains.reset();
   }

   public YoFootPositionGains getPositionGains()
   {
      return positionGains;
   }

   public YoFootOrientationGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionProportionalGains(double proportionalGainXY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainXY, proportionalGainZ);
   }

   public void setPositionDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      positionGains.setDerivativeGains(derivativeGainXY, derivativeGainZ);
   }

   public void setPositionDampingRatio(double dampingRatio)
   {
      positionGains.setDampingRatio(dampingRatio);
   }

   public void setPositionMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      positionGains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
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

   public void setOrientationMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      orientationGains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      positionGains.createDerivativeGainUpdater(updateNow);
      orientationGains.createDerivativeGainUpdater(updateNow);
   }
}

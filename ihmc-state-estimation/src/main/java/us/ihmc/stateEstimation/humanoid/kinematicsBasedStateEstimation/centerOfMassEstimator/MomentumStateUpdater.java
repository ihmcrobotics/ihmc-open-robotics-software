package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface MomentumStateUpdater
{
   void initialize();

   void update();

   YoRegistry getRegistry();
}

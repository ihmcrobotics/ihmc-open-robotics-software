package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface MomentumStateUpdater
{
   void initialize();

   void update();

   YoRegistry getRegistry();

   default YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}

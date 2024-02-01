package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface MomentumStateUpdater extends SCS2YoGraphicHolder
{
   void initialize();

   void update();

   YoRegistry getRegistry();

   @Override
   default YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}

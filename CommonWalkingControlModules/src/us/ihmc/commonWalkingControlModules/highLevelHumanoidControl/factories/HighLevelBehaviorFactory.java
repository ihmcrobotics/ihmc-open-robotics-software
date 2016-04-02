package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public interface HighLevelBehaviorFactory
{
   public abstract HighLevelBehavior createHighLevelBehavior(HighLevelControlManagerFactory variousWalkingManagers, MomentumBasedController momentumBasedController);

   public abstract boolean isTransitionToBehaviorRequested();
}

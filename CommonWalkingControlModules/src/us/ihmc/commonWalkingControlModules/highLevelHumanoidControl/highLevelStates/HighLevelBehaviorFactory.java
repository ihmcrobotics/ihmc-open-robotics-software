package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public interface HighLevelBehaviorFactory
{
   public abstract HighLevelBehavior createHighLevelBehavior(MomentumBasedController momentumBasedController,
         ICPAndMomentumBasedController icpAndMomentumBasedController);

   public abstract boolean isTransitionToBehaviorRequested();
}

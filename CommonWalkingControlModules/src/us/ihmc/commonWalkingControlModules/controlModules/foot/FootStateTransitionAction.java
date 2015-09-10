package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;

public class FootStateTransitionAction implements StateTransitionAction
{
   private final FootControlHelper footControlHelper;
   private final BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState;

   public FootStateTransitionAction(FootControlHelper footControlHelper, BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState)
   {
      this.footControlHelper = footControlHelper;
      this.waitSingularityEscapeBeforeTransitionToNextState = waitSingularityEscapeBeforeTransitionToNextState;
   }

   public void doTransitionAction()
   {
      footControlHelper.resetAccelerationControlModule();
      footControlHelper.setRequestedStateAsProcessed();
      footControlHelper.resetSingularityEscape();
      waitSingularityEscapeBeforeTransitionToNextState.set(false);
   }
}

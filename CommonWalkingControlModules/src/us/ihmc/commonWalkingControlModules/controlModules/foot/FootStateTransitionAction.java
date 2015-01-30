package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;

public class FootStateTransitionAction implements StateTransitionAction
{
   private final FootControlHelper fooconControlHelper;
   private final BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState;

   public FootStateTransitionAction(FootControlHelper fooconControlHelper, BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState)
   {
      this.fooconControlHelper = fooconControlHelper;
      this.waitSingularityEscapeBeforeTransitionToNextState = waitSingularityEscapeBeforeTransitionToNextState;
   }

   public void doTransitionAction()
   {
      fooconControlHelper.setRequestedStateAsProcessed();
      fooconControlHelper.resetSingularityEscape();
      waitSingularityEscapeBeforeTransitionToNextState.set(false);
   }
}

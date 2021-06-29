package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.PushRecoveryStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveryTransferState;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class ContinuePushRecoveryWithNextStepCondition implements StateTransitionCondition
{
   private final RecoveryTransferState transferState;
   private final PushRecoveryStateEnum nextSwingEnum;
   private final MultiStepPushRecoveryController pushRecoveryControlModule;

   public ContinuePushRecoveryWithNextStepCondition(RecoveryTransferState transferState, PushRecoveryStateEnum nextSwingEnum,
                                                    MultiStepPushRecoveryController pushRecoveryControlModule)
   {
      this.transferState = transferState;
      this.nextSwingEnum = nextSwingEnum;
      this.pushRecoveryControlModule = pushRecoveryControlModule;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!transferState.isDone(timeInState))
         return false;

      if  (!pushRecoveryControlModule.isRobotFallingFromDoubleSupport())
         return false;

      return pushRecoveryControlModule.getSwingSideForRecovery() == nextSwingEnum.getSupportSide().getOppositeSide();
   }
}
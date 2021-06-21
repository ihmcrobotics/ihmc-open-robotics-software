package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StandingToSwingCondition implements StateTransitionCondition
{
   private final RobotSide swingSide;

   private final MultiStepPushRecoveryController pushRecoveryControlModule;

   public StandingToSwingCondition(RobotSide singleSupportStateSwingSide,
                                   MultiStepPushRecoveryController pushRecoveryControlModule)
   {
      this.pushRecoveryControlModule = pushRecoveryControlModule;
      swingSide = singleSupportStateSwingSide;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      boolean isRobotFalling = pushRecoveryControlModule.isRobotFallingFromDoubleSupport();

      if (!isRobotFalling)
         return false;

      return swingSide == pushRecoveryControlModule.getSwingSideForRecovery();
   }
}
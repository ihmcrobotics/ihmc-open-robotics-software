package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class StandingToSwingCondition implements StateTransitionCondition
{
   private final RobotSide swingSide;

   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;

   public StandingToSwingCondition(RobotSide singleSupportStateSwingSide,
                                   MultiStepPushRecoveryControlModule pushRecoveryControlModule)
   {
      this.pushRecoveryControlModule = pushRecoveryControlModule;
      swingSide = singleSupportStateSwingSide;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      RobotSide suggestedSwingSide = pushRecoveryControlModule.isRobotFallingFromDoubleSupport();
      boolean isRobotFalling = suggestedSwingSide != null;

      if (!isRobotFalling)
         return false;

      return swingSide == suggestedSwingSide;
   }
}
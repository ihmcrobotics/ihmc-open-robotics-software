package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class DoubSuppToSingSuppCond4DistRecov implements StateTransitionCondition
{
   private final RobotSide swingSide;
   private final BalanceManager balanceManager;

   public DoubSuppToSingSuppCond4DistRecov(RobotSide singleSupportStateSwingSide, BalanceManager balanceManager)
   {
      swingSide = singleSupportStateSwingSide;
      this.balanceManager = balanceManager;
   }

   @Override
   public boolean checkCondition()
   {
      if (!balanceManager.isPushRecoveryEnabled())
         return false;

      RobotSide suggestedSwingSide = balanceManager.isRobotFallingFromDoubleSupport();
      boolean isRobotFalling = suggestedSwingSide != null;

      if (!isRobotFalling)
         return false;

      boolean switchToSingleSupport = swingSide == suggestedSwingSide;

      if (switchToSingleSupport)
         balanceManager.prepareForDoubleSupportPushRecovery();

      return switchToSingleSupport;
   }
}
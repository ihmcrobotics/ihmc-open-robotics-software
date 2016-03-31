package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class DoubleSupportToSingleSupportConditionForDisturbanceRecovery implements StateTransitionCondition
{
   private final RobotSide transferToSide;
   private final BalanceManager balanceManager;

   public DoubleSupportToSingleSupportConditionForDisturbanceRecovery(RobotSide singleSupportStateSupportSide, BalanceManager balanceManager)
   {
      transferToSide = singleSupportStateSupportSide;
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

      boolean switchToSingleSupport = transferToSide != suggestedSwingSide;

      if (switchToSingleSupport)
         balanceManager.prepareForDoubleSupportPushRecovery();

      return switchToSingleSupport;
   }
}
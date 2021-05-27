package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.commons.FormattingTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class PushRecoveryState implements State
{
   protected final YoRegistry registry;
   private final PushRecoveryStateEnum recoveryStateEnum;
   private PushRecoveryStateEnum previousRecoveryStateEnum = null;

   public PushRecoveryState(PushRecoveryStateEnum stateEnum, YoRegistry parentRegistry)
   {
      this.recoveryStateEnum = stateEnum;
      registry = new YoRegistry(FormattingTools.underscoredToCamelCase(stateEnum.toString(), true));
      parentRegistry.addChild(registry);
   }

   public boolean isDoubleSupportState()
   {
      return getStateEnum().isDoubleSupport();
   }

   public boolean isSingleSupportState()
   {
      return getStateEnum().isSingleSupport();
   }

   public RobotSide getSupportSide()
   {
      return getStateEnum().getSupportSide();
   }

   public RobotSide getTransferToSide()
   {
      return getStateEnum().getTransferToSide();
   }

   public void handleFootLoadBearingCommand(FootLoadBearingCommand command)
   {
      // Override in state that can handle EndEffectorLoadBearingCommand
   }

   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return false;
   }

   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return false;
   }

   public PushRecoveryStateEnum getStateEnum()
   {
      return recoveryStateEnum;
   }

   public void setPreviousWalkingStateEnum(PushRecoveryStateEnum previousRecoveryStateEnum)
   {
      this.previousRecoveryStateEnum = previousRecoveryStateEnum;
   }

   public PushRecoveryStateEnum getPreviousWalkingStateEnum()
   {
      return previousRecoveryStateEnum;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

}

package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commons.FormattingTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class WalkingState implements State
{
   protected final YoRegistry registry;
   private final WalkingStateEnum walkingStateEnum;
   private WalkingStateEnum previousWalkingStateEnum = null;

   public WalkingState(WalkingStateEnum stateEnum, YoRegistry parentRegistry)
   {
      this.walkingStateEnum = stateEnum;
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

   public WalkingStateEnum getStateEnum()
   {
      return walkingStateEnum;
   }

   public void setPreviousWalkingStateEnum(WalkingStateEnum previousWalkingStateEnum)
   {
      this.previousWalkingStateEnum = previousWalkingStateEnum;
   }

   public WalkingStateEnum getPreviousWalkingStateEnum()
   {
      return previousWalkingStateEnum;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

}

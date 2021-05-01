package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.FormattingTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class JumpingState implements State
{
   protected final YoRegistry registry;
   private final JumpingStateEnum jumpingStateEnum;
   private JumpingStateEnum previousJumpingStateEnum = null;

   public JumpingState(JumpingStateEnum stateEnum, YoRegistry parentRegistry)
   {
      this.jumpingStateEnum = stateEnum;
      registry = new YoRegistry(FormattingTools.underscoredToCamelCase(stateEnum.toString(), true));
      parentRegistry.addChild(registry);
   }

   public JumpingStateEnum getStateEnum()
   {
      return jumpingStateEnum;
   }

   public void setPreviousJumpingStateEnum(JumpingStateEnum previousJumpingStateEnum)
   {
      this.previousJumpingStateEnum = previousJumpingStateEnum;
   }

   public JumpingStateEnum getPreviousJumpingStateEnum()
   {
      return previousJumpingStateEnum;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

}

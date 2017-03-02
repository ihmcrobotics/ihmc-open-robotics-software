package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EndEffectorLoadBearingCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.tools.FormattingTools;

public abstract class WalkingState extends FinishableState<WalkingStateEnum>
{
   protected final YoVariableRegistry registry;

   public WalkingState(WalkingStateEnum stateEnum, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      registry = new YoVariableRegistry(FormattingTools.underscoredToCamelCase(stateEnum.toString(), true));
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

   public void handleEndEffectorLoadBearingCommand(EndEffectorLoadBearingCommand command)
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

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

}

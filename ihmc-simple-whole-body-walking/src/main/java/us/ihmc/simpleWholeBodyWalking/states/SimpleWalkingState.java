package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commons.FormattingTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class SimpleWalkingState implements State
{
   protected final YoVariableRegistry registry;
   private final SimpleWalkingStateEnum walkingStateEnum;
   private SimpleWalkingStateEnum previousWalkingStateEnum = null;

   public SimpleWalkingState(SimpleWalkingStateEnum stateEnum, YoVariableRegistry parentRegistry)
   {
      this.walkingStateEnum = stateEnum;
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

   public RobotSide getTransferToSide()
   {
      return getStateEnum().getTransferToSide();
   }

   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return false;
   }

   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return false;
   }

   public SimpleWalkingStateEnum getStateEnum()
   {
      return walkingStateEnum;
   }

   public void setPreviousWalkingStateEnum(SimpleWalkingStateEnum previousWalkingStateEnum)
   {
      this.previousWalkingStateEnum = previousWalkingStateEnum;
   }

   public SimpleWalkingStateEnum getPreviousWalkingStateEnum()
   {
      return previousWalkingStateEnum;
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

}

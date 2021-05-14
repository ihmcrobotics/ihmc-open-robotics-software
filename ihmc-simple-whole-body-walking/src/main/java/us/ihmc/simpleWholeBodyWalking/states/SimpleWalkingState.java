package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commons.FormattingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class SimpleWalkingState implements State
{
   protected final YoRegistry registry;
   private final SimpleWalkingStateEnum walkingStateEnum;
   private SimpleWalkingStateEnum previousWalkingStateEnum = null;

   public SimpleWalkingState(SimpleWalkingStateEnum stateEnum, YoRegistry parentRegistry)
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

   public YoRegistry getRegistry()
   {
      return registry;
   }

}

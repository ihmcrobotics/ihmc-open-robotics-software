package us.ihmc.behaviors.behaviorTree.action.actions;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;

import java.util.Arrays;

public class AbilityHandActionExecutor extends ActionNodeExecutor<AbilityHandActionState, AbilityHandActionDefinition>
{
   public AbilityHandActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Executing Ability Hand action for side: {} with grip {}", definition.getSide(), definition.getGrip());

      String identifier = HandInterface.getSimpleIdentifier(robotModel.getSimpleRobotName(), definition.getSide(), HandType.ABILITY_HAND);

      if (abilityHandCommunication.getAvailableHands().contains(identifier))
      {
         AbilityHandCommand command = abilityHandCommunication.getCommand(identifier);
         Arrays.fill(command.getGoalVelocities(), 30.0f);
         command.setControlMode(definition.getControlMode().toByte());
         if (definition.getControlMode() == ControlMode.GRIP)
         {
            command.setGrip(definition.getGrip().toByte());
         }
         else
         {
            for (int i = 0; i < 5; i++)
               command.getGoalPositions()[i] = definition.getGoalPositions().getValueReadOnly(i);
            command.getGoalPositions()[5] = -definition.getGoalPositions().getValueReadOnly(5);
         }
         for (int i = 0; i < 6; i++)
            command.getGoalVelocities()[i] = definition.getGoalVelocities().getValueReadOnly(i);
         abilityHandCommunication.publishCommand(identifier);
      }
      else
      {
         state.setFailed(true);
      }

      state.setIsExecuting(false);
   }
}

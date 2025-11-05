package us.ihmc.behaviors.behaviorTree.action.actions;

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
         Arrays.fill(abilityHandCommunication.getCommand(identifier).getGoalVelocities(), 30.0f);
         abilityHandCommunication.getCommand(identifier).setControlMode(ControlMode.GRIP.toByte());
         abilityHandCommunication.getCommand(identifier).setGrip(definition.getGrip().toByte());
         abilityHandCommunication.publishCommand(identifier);
      }
      else
      {
         state.setFailed(true);
      }

      state.setIsExecuting(false);
   }
}

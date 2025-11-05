package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;

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

      String handIdentifier = HandInterface.getSimpleIdentifier(robotModel.getSimpleRobotName(), definition.getSide(), HandType.ABILITY_HAND);

      if (abilityHandCommunication.getAvailableHands().contains(handIdentifier))
      {
         abilityHandCommunication.getCommand(handIdentifier).setGrip(definition.getGrip().toByte());
         abilityHandCommunication.publishCommand(handIdentifier);
      }
      else
      {
         state.setFailed(true);
         state.setIsExecuting(false);
      }
   }
}

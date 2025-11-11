package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;

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

      state.getLogger().info("Executing Ability Hand action for side: {}", definition.getSide());

      // TODO: Implement Ability Hand action execution
   }
}

package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition.MimicActionType;

/**
 * Plays a catalog mimic through the same player speech uses. The leaf does not wait on this thread:
 * {@link #triggerExecution()} starts the maneuver and later ticks read the result.
 * An exit-policy node completes immediately. The controller maneuver already includes its own exit.
 */
public class MimicActionExecutor extends ActionNodeExecutor<MimicActionState, MimicActionDefinition>
{
   private long mimicRequestId;

   public MimicActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new MimicActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();
      if (definition.getMimicActionType().getValue() == MimicActionType.EXIT_POLICY)
         return;

      BehaviorTreeExecutor tree = rootNode.getTree();
      mimicRequestId = tree.startNestedMimic(definition.getMimicFileName());
      finishIfTerminal(state, mimicRequestId, tree.nestedMimicPhase(mimicRequestId));
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (definition.getMimicActionType().getValue() == MimicActionType.EXIT_POLICY)
      {
         state.setIsExecuting(false);
         return;
      }
      finishIfTerminal(state, mimicRequestId, rootNode.getTree().nestedMimicPhase(mimicRequestId));
   }
}

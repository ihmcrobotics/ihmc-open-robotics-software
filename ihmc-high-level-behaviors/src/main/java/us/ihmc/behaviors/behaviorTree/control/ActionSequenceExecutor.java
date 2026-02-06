package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   public ActionSequenceExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new ActionSequenceState(id, rootNode.getState()), rootNode);
   }
}

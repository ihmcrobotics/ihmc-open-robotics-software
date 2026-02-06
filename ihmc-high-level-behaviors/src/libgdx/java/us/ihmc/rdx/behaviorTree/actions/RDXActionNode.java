package us.ihmc.rdx.behaviorTree.actions;

import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXActionNode<S extends ActionNodeState<D>,
                                    D extends ActionNodeDefinition>
      extends RDXLeafNode<S, D>
{
   private final RDXActionProgressWidgets progressWidgets = new RDXActionProgressWidgets(this);

   public RDXActionNode(S state, RDXBehaviorTreeRootNode rootNode)
   {
      super(state, rootNode);
   }

   public RDXActionProgressWidgets getProgressWidgets()
   {
      return progressWidgets;
   }
}

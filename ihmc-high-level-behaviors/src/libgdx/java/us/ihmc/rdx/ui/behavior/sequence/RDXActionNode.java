package us.ihmc.rdx.ui.behavior.sequence;

import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXActionNode<S extends ActionNodeState<D>,
                                    D extends ActionNodeDefinition>
      extends RDXLeafNode<S, D>
{
   private final S state;
   private final D definition;
   private final RDXActionProgressWidgets progressWidgets = new RDXActionProgressWidgets(this);

   public RDXActionNode(S state)
   {
      super(state);

      this.state = state;
      definition = getDefinition();
   }

   public RDXActionProgressWidgets getProgressWidgets()
   {
      return progressWidgets;
   }
}

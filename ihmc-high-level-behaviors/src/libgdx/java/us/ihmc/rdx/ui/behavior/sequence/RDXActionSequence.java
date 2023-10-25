package us.ihmc.rdx.ui.behavior.sequence;

import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXActionSequence extends RDXBehaviorTreeNode<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;

   public RDXActionSequence(long id)
   {
      state = new ActionSequenceState(id);
   }

   @Override
   public ActionSequenceState getState()
   {
      return state;
   }
}

package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;

   public ActionSequenceExecutor(long id)
   {
      state = new ActionSequenceState(id);
   }

   @Override
   public void clock()
   {

   }

   @Override
   public ActionSequenceState getState()
   {
      return state;
   }
}

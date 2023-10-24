package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class BehaviorActionSequenceExecutor extends BehaviorTreeNodeExecutor<BehaviorActionSequenceState, BehaviorActionSequenceDefinition>
{
   private final BehaviorActionSequenceState state;

   public BehaviorActionSequenceExecutor(long id)
   {
      state = new BehaviorActionSequenceState(id);
   }

   @Override
   public void clock()
   {

   }

   @Override
   public BehaviorActionSequenceState getState()
   {
      return state;
   }
}

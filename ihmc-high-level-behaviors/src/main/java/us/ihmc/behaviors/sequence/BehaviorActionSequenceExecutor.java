package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorActionSequenceExecutor extends BehaviorTreeNodeExecutor
{
   private final BehaviorActionSequenceDefinition definition = new BehaviorActionSequenceDefinition();
   private final BehaviorActionSequenceState state;


   public BehaviorActionSequenceExecutor(long id)
   {
      state = new BehaviorActionSequenceState(id, definition);
   }

   @Override
   public void clock()
   {

   }

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return definition;
   }

   @Override
   public BehaviorTreeNodeState getState()
   {
      return state;
   }
}

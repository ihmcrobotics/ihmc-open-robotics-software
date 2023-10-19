package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorActionSequenceState extends BehaviorTreeNodeState
{
   private final BehaviorActionSequenceDefinition definition;

   public BehaviorActionSequenceState(long id, BehaviorActionSequenceDefinition definition)
   {
      super(id, definition);
      this.definition = definition;
   }
}

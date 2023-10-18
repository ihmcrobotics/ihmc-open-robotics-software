package us.ihmc.behaviors.behaviorTree;

public class BehaviorTreeExecutor
{
   private final BehaviorTreeState behaviorTreeState;

   public BehaviorTreeExecutor(BehaviorTreeState behaviorTreeState)
   {
      this.behaviorTreeState = behaviorTreeState;

      new BehaviorTreeNodeExecutor()

   }
}

package us.ihmc.behaviors.behaviorTree;

public class BehaviorTreeExecutor
{
   private final BehaviorTreeState behaviorTreeState;
   private final BehaviorTreeNodeExecutor rootNode;

   public BehaviorTreeExecutor(BehaviorTreeState behaviorTreeState)
   {
      this.behaviorTreeState = behaviorTreeState;

   }

   public void update()
   {
      rootNode.clock();

      rootNode.tick();

   }

   public BehaviorTreeState getBehaviorTreeState()
   {
      return behaviorTreeState;
   }
}

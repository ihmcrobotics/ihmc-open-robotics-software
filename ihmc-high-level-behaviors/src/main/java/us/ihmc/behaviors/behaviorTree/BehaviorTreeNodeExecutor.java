package us.ihmc.behaviors.behaviorTree;

public abstract class BehaviorTreeNodeExecutor implements BehaviorTreeNodeStateSupplier
{
   public void update()
   {
      getState().clock();
   }
}

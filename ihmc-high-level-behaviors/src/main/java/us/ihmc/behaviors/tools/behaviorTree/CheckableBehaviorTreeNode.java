package us.ihmc.behaviors.tools.behaviorTree;

public abstract class CheckableBehaviorTreeNode implements BehaviorTreeNode
{
   private BehaviorTreeNodeStatus status = null;

   protected abstract BehaviorTreeNodeStatus tickInternal();

   @Override
   public final BehaviorTreeNodeStatus tick() {
      status = tickInternal();
      return status;
   }

   public BehaviorTreeNodeStatus getPreviousStatus() {
      return status;
   }
}

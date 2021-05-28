package us.ihmc.behaviors.tools.behaviorTree;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNode implements BehaviorTreeNodeBasics
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

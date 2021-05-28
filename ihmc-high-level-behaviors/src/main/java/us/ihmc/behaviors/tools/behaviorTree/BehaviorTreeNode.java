package us.ihmc.behaviors.tools.behaviorTree;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNode implements BehaviorTreeNodeBasics
{
   private BehaviorTreeNodeStatus status = null;
   private long time = -1;

   protected abstract BehaviorTreeNodeStatus tickInternal();

   @Override
   public final BehaviorTreeNodeStatus tick() {
      status = tickInternal();
      time = System.currentTimeMillis();
      return status;
   }

   public BehaviorTreeNodeStatus getPreviousStatus() {
      return status;
   }

   public long lastTickMillis() {
      return time;
   }

   public String getName() {
      return null;
   }
}

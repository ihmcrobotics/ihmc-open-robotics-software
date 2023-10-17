package us.ihmc.behaviors.behaviorTree;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public class LocalOnlyBehaviorTreeNodeState extends BehaviorTreeNodeState
{
   /** The current status of the behavior tree node. */
   private BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.SUCCESS;
   private final BehaviorTreeNodeDefinition definition = new BehaviorTreeNodeDefinition();

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return definition;
   }

   public void setStatus(BehaviorTreeNodeStatus status)
   {
      this.status = status;
   }

   /**
    * @return The node's status from the last time it was ticked.
    *         This will be null if the node hasn't been ticked yet.
    */
   public BehaviorTreeNodeStatus getStatus()
   {
      return status;
   }
}

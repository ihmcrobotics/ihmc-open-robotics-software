package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public class LocalOnlyBehaviorTreeNodeState extends BehaviorTreeNodeState<BehaviorTreeNodeDefinition>
{
   /** The current status of the behavior tree node. */
   private BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.SUCCESS;

   public LocalOnlyBehaviorTreeNodeState()
   {
      super(0, new BehaviorTreeNodeDefinition(), new CRDTInfo(ROS2ActorDesignation.ROBOT, 5));
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

package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public class LocalOnlyBehaviorTreeNodeState extends BehaviorTreeNodeState<BehaviorTreeNodeDefinition>
{
   /** The current status of the behavior tree node. */
   private BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.SUCCESS;

   public LocalOnlyBehaviorTreeNodeState(CRDTInfo crdtInfo)
   {
      super(0, new BehaviorTreeNodeDefinition(crdtInfo, new WorkspaceResourceDirectory(LocalOnlyBehaviorTreeNodeState.class)), crdtInfo);
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

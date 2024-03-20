package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalState extends BehaviorTreeNodeState<DoorTraversalDefinition>
{
   public DoorTraversalState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new DoorTraversalDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node)
   {
      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   public void toMessage(DoorTraversalStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(DoorTraversalStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

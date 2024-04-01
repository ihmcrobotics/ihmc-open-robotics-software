package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalDefinition extends BehaviorTreeNodeDefinition
{
   // Seems to be nothing special here so far TODO Does that mean we delete it?

   public DoorTraversalDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(DoorTraversalDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(DoorTraversalDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

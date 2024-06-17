package us.ihmc.behaviors.roomExploration;

import behavior_msgs.msg.dds.RoomExplorationDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RoomExplorationDefinition extends BehaviorTreeNodeDefinition
{
   public RoomExplorationDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(RoomExplorationDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(RoomExplorationDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

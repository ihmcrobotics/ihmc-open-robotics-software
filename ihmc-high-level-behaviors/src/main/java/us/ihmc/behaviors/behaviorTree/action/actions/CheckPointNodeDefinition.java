package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.CheckPointNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.action.LeafNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class CheckPointNodeDefinition extends LeafNodeDefinition
{
   public CheckPointNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(CheckPointNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(CheckPointNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

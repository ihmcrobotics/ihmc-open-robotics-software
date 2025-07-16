package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.FallbackNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FallbackNodeDefinition extends BehaviorTreeNodeDefinition
{
   public FallbackNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(FallbackNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(FallbackNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

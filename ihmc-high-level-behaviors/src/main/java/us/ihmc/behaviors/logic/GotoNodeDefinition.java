package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.GotoNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTGlobalInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class GotoNodeDefinition extends BehaviorTreeNodeDefinition
{
   public GotoNodeDefinition(CRDTGlobalInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(GotoNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(GotoNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

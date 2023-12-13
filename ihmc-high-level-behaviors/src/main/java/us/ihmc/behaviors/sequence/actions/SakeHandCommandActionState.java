package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class SakeHandCommandActionState extends ActionNodeState<SakeHandCommandActionDefinition>
{
   public SakeHandCommandActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new SakeHandCommandActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

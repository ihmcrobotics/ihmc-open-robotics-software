package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandWrenchActionState extends ActionNodeState<HandWrenchActionDefinition>
{
   public HandWrenchActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new HandWrenchActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   public void toMessage(HandWrenchActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

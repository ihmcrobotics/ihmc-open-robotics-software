package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
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
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}

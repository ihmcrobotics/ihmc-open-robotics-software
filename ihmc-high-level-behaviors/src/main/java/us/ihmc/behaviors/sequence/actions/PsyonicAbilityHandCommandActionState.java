package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PsyonicAbilityHandCommandActionState extends ActionNodeState<PsyonicAbilityHandCommandActionDefinition>
{
   public PsyonicAbilityHandCommandActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new PsyonicAbilityHandCommandActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   public void toMessage(PsyonicAbilityHandCommandActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(PsyonicAbilityHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

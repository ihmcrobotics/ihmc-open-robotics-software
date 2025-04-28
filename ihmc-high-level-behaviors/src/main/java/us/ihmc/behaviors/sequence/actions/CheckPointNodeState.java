package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.CheckPointNodeStateMessage;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class CheckPointNodeState extends LeafNodeState<CheckPointNodeDefinition>
{
   public CheckPointNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new CheckPointNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(CheckPointNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(CheckPointNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      definition.fromMessage(message.getDefinition());
   }
}

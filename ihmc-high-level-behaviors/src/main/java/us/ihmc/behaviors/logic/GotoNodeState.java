package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.GotoNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class GotoNodeState extends BehaviorTreeNodeState<GotoNodeDefinition>
{
   public GotoNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new GotoNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(GotoNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(GotoNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

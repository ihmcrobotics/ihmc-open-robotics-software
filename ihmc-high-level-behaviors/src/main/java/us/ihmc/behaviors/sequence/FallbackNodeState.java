package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.FallbackNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FallbackNodeState extends BehaviorTreeNodeState<FallbackNodeDefinition>
{
   public FallbackNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new FallbackNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(FallbackNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FallbackNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

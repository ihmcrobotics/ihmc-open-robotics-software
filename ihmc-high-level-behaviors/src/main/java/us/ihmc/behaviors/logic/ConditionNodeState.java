package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeState extends BehaviorTreeNodeState<ConditionNodeDefinition>
{
   public ConditionNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ConditionNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeState extends LeafNodeState<ConditionNodeDefinition>
{
   private final CRDTBidirectionalLong count;

   public ConditionNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ConditionNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      count = new CRDTBidirectionalLong(definition, 0);
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setCount(count.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
      
      count.fromMessage(message.getCount());
   }

   public CRDTBidirectionalLong getCount()
   {
      return count;
   }
}

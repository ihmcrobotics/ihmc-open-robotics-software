package us.ihmc.behaviors.logic.condition;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;

public class CounterConditionState
{
   private final CRDTBidirectionalLong count;

   public CounterConditionState(ConditionNodeDefinition definition)
   {
      count = new CRDTBidirectionalLong(definition, 0);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.setCount(count.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      count.fromMessage(message.getCount());
   }

   public CRDTBidirectionalLong getCount()
   {
      return count;
   }
}

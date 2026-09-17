package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.ConditionNodeStateMessage;
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
      message.setCount((int) count.toMessage());
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

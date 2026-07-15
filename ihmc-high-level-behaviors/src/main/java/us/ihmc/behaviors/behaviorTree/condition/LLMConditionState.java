package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.ConditionNodeStateMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalNotification;

public class LLMConditionState
{
   private final CRDTBidirectionalNotification resetContextRequest;

   public LLMConditionState(ConditionNodeDefinition definition)
   {
      resetContextRequest = new CRDTBidirectionalNotification(definition);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.setRequestResetContext(resetContextRequest.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      resetContextRequest.fromMessage(message.getRequestResetContext());
   }

   public boolean pollResetContextRequested()
   {
      return resetContextRequest.poll();
   }

   public boolean getResetContextRequested()
   {
      return resetContextRequest.peek();
   }

   public void setResetContextRequested()
   {
      resetContextRequest.set();
   }
}

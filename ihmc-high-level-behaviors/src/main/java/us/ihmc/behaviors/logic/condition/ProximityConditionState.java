package us.ihmc.behaviors.logic.condition;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;

public class ProximityConditionState
{
   private final CRDTBidirectionalDouble currentDistance;

   public ProximityConditionState(ConditionNodeDefinition definition)
   {
      currentDistance = new CRDTBidirectionalDouble(definition, -1.0);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.setCurrentDistance(currentDistance.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      currentDistance.fromMessage(message.getCurrentDistance());
   }

   public CRDTBidirectionalDouble getCurrentDistance()
   {
      return currentDistance;
   }

   public void setCurrentDistance(double distance)
   {
      currentDistance.setValue(distance);
   }
}

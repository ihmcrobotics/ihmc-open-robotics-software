package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;

public class ProximityConditionState
{
   private final CRDTBidirectionalDouble currentDistance;
   private final CRDTBidirectionalBoolean missingFrame;

   public ProximityConditionState(ConditionNodeDefinition definition)
   {
      currentDistance = new CRDTBidirectionalDouble(definition, -1.0);
      missingFrame = new CRDTBidirectionalBoolean(definition, false);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      message.setCurrentDistance(currentDistance.toMessage());
      message.setMissingFrame(missingFrame.toMessage());
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      currentDistance.fromMessage(message.getCurrentDistance());
      missingFrame.fromMessage(message.getMissingFrame());
   }

   public CRDTBidirectionalDouble getCurrentDistance()
   {
      return currentDistance;
   }

   public CRDTBidirectionalBoolean getMissingFrame()
   {
      return missingFrame;
   }

   public void setCurrentDistance(double distance)
   {
      currentDistance.setValue(distance);
   }

   public void setMissingFrame(boolean value)
   {
      missingFrame.setValue(value);
   }
}
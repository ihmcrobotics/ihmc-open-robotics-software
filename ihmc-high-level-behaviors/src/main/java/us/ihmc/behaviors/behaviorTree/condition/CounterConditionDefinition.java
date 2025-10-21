package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

public class CounterConditionDefinition
{
   private final CRDTBidirectionalLong countTo;

   private long onDiskCountTo = 0;

   public CounterConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      countTo = new CRDTBidirectionalLong(latestTimestampModifiable, 0);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("countTo", countTo.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      countTo.setValue(jsonNode.get("countTo").longValue());
   }

   public void setOnDiskFields()
   {
      onDiskCountTo = countTo.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      countTo.setValue(onDiskCountTo);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= countTo.getValue() == onDiskCountTo;

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setCountTo(countTo.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      countTo.fromMessage(message.getCountTo());
   }

   public CRDTBidirectionalLong getCountTo()
   {
      return countTo;
   }
}

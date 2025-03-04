package us.ihmc.behaviors.logic.condition;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

public class LLMConditionDefinition
{
   public static final String DEFAULT_PROMPT = """
         TODO
         """;

   private final CRDTBidirectionalString prompt;

   private String onDiskPrompt;

   public LLMConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      prompt = new CRDTBidirectionalString(latestTimestampModifiable, DEFAULT_PROMPT);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("prompt", prompt.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      prompt.setValue(jsonNode.get("prompt").textValue());
   }

   public void setOnDiskFields()
   {
      onDiskPrompt = prompt.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      prompt.setValue(onDiskPrompt);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= prompt.getValue().equals(onDiskPrompt);

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setPrompt(prompt.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      prompt.fromMessage(message.getPromptAsString());
   }

   public CRDTBidirectionalString getPrompt()
   {
      return prompt;
   }
}

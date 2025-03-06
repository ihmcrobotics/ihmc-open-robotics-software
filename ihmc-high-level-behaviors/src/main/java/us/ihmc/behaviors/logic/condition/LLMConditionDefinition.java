package us.ihmc.behaviors.logic.condition;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.llama.Llama;

public class LLMConditionDefinition
{
   public static final String DEFAULT_PROMPT = """
         TODO
         """;

   private final CRDTBidirectionalBoolean resetContextEachRun;
   private final CRDTBidirectionalBoolean injectBehaviorState;
   private final CRDTBidirectionalBoolean injectEnvironmentState;
   private final CRDTBidirectionalString system;
   private final CRDTBidirectionalString prompt;

   private boolean onDiskResetContextEachRun;
   private boolean onDiskInjectBehaviorState;
   private boolean onDiskInjectEnvironmentState;
   private String onDiskSystem;
   private String onDiskPrompt;

   public LLMConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      resetContextEachRun = new CRDTBidirectionalBoolean(latestTimestampModifiable, true);
      injectBehaviorState = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      injectEnvironmentState = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      system = new CRDTBidirectionalString(latestTimestampModifiable, Llama.CHAT_WITH_LLAMA);
      prompt = new CRDTBidirectionalString(latestTimestampModifiable, DEFAULT_PROMPT);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("resetContextEachRun", resetContextEachRun.getValue());
      jsonNode.put("injectBehaviorState", injectBehaviorState.getValue());
      jsonNode.put("injectEnvironmentState", injectEnvironmentState.getValue());
      jsonNode.put("system", system.getValue());
      jsonNode.put("prompt", prompt.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      resetContextEachRun.setValue(jsonNode.get("resetContextEachRun").booleanValue());
      injectBehaviorState.setValue(jsonNode.get("injectBehaviorState").booleanValue());
      injectEnvironmentState.setValue(jsonNode.get("injectEnvironmentState").booleanValue());
      system.setValue(jsonNode.get("system").textValue());
      prompt.setValue(jsonNode.get("prompt").textValue());
   }

   public void setOnDiskFields()
   {
      onDiskResetContextEachRun = resetContextEachRun.getValue();
      onDiskInjectBehaviorState = injectBehaviorState.getValue();
      onDiskInjectEnvironmentState = injectEnvironmentState.getValue();
      onDiskSystem = system.getValue();
      onDiskPrompt = prompt.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      resetContextEachRun.setValue(onDiskResetContextEachRun);
      injectBehaviorState.setValue(onDiskInjectBehaviorState);
      injectEnvironmentState.setValue(onDiskInjectEnvironmentState);
      system.setValue(onDiskSystem);
      prompt.setValue(onDiskPrompt);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= resetContextEachRun.getValue() == (onDiskResetContextEachRun);
      unchanged &= injectBehaviorState.getValue() == (onDiskInjectBehaviorState);
      unchanged &= injectEnvironmentState.getValue() == (onDiskInjectEnvironmentState);
      unchanged &= system.getValue().equals(onDiskSystem);
      unchanged &= prompt.getValue().equals(onDiskPrompt);

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setResetContextEachRun(resetContextEachRun.toMessage());
      message.setInjectBehaviorState(injectBehaviorState.toMessage());
      message.setInjectEnvironmentState(injectEnvironmentState.toMessage());
      message.setSystem(system.toMessage());
      message.setPrompt(prompt.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      resetContextEachRun.fromMessage(message.getResetContextEachRun());
      injectBehaviorState.fromMessage(message.getInjectBehaviorState());
      injectEnvironmentState.fromMessage(message.getInjectEnvironmentState());
      system.fromMessage(message.getSystemAsString());
      prompt.fromMessage(message.getPromptAsString());
   }

   public CRDTBidirectionalBoolean getResetContextEachRun()
   {
      return resetContextEachRun;
   }

   public CRDTBidirectionalBoolean getInjectBehaviorState()
   {
      return injectBehaviorState;
   }

   public CRDTBidirectionalBoolean getInjectEnvironmentState()
   {
      return injectEnvironmentState;
   }

   public CRDTBidirectionalString getSystem()
   {
      return system;
   }

   public CRDTBidirectionalString getPrompt()
   {
      return prompt;
   }
}

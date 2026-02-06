package us.ihmc.behaviors.behaviorTree.condition;

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
   private final CRDTBidirectionalBoolean matchIsSuccess;
   private final CRDTBidirectionalString system;
   private final CRDTBidirectionalString prompt;
   private final CRDTBidirectionalString responseMatcher;

   private boolean onDiskResetContextEachRun;
   private boolean onDiskInjectBehaviorState;
   private boolean onDiskInjectEnvironmentState;
   private boolean onDiskMatchIsSuccess;
   private String onDiskSystem;
   private String onDiskPrompt;
   private String onDiskResponseMatcher;

   public LLMConditionDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      resetContextEachRun = new CRDTBidirectionalBoolean(latestTimestampModifiable, true);
      injectBehaviorState = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      injectEnvironmentState = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      matchIsSuccess = new CRDTBidirectionalBoolean(latestTimestampModifiable, true);
      system = new CRDTBidirectionalString(latestTimestampModifiable, Llama.CHAT_WITH_LLAMA);
      prompt = new CRDTBidirectionalString(latestTimestampModifiable, DEFAULT_PROMPT);
      responseMatcher = new CRDTBidirectionalString(latestTimestampModifiable, "failure");
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("resetContextEachRun", resetContextEachRun.getValue());
      jsonNode.put("injectBehaviorState", injectBehaviorState.getValue());
      jsonNode.put("injectEnvironmentState", injectEnvironmentState.getValue());
      jsonNode.put("matchIsSuccess", matchIsSuccess.getValue());
      jsonNode.put("system", system.getValue());
      jsonNode.put("prompt", prompt.getValue());
      jsonNode.put("responseMatcher", responseMatcher.getValue());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      resetContextEachRun.setValue(jsonNode.get("resetContextEachRun").booleanValue());
      injectBehaviorState.setValue(jsonNode.get("injectBehaviorState").booleanValue());
      injectEnvironmentState.setValue(jsonNode.get("injectEnvironmentState").booleanValue());
      matchIsSuccess.setValue(jsonNode.get("matchIsSuccess").booleanValue());
      system.setValue(jsonNode.get("system").textValue());
      prompt.setValue(jsonNode.get("prompt").textValue());
      responseMatcher.setValue(jsonNode.get("responseMatcher").textValue());
   }

   public void setOnDiskFields()
   {
      onDiskResetContextEachRun = resetContextEachRun.getValue();
      onDiskInjectBehaviorState = injectBehaviorState.getValue();
      onDiskInjectEnvironmentState = injectEnvironmentState.getValue();
      onDiskMatchIsSuccess = matchIsSuccess.getValue();
      onDiskSystem = system.getValue();
      onDiskPrompt = prompt.getValue();
      onDiskResponseMatcher = responseMatcher.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      resetContextEachRun.setValue(onDiskResetContextEachRun);
      injectBehaviorState.setValue(onDiskInjectBehaviorState);
      injectEnvironmentState.setValue(onDiskInjectEnvironmentState);
      matchIsSuccess.setValue(onDiskMatchIsSuccess);
      system.setValue(onDiskSystem);
      prompt.setValue(onDiskPrompt);
      responseMatcher.setValue(onDiskResponseMatcher);
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= resetContextEachRun.getValue() == (onDiskResetContextEachRun);
      unchanged &= injectBehaviorState.getValue() == (onDiskInjectBehaviorState);
      unchanged &= injectEnvironmentState.getValue() == (onDiskInjectEnvironmentState);
      unchanged &= matchIsSuccess.getValue() == (onDiskMatchIsSuccess);
      unchanged &= system.getValue().equals(onDiskSystem);
      unchanged &= prompt.getValue().equals(onDiskPrompt);
      unchanged &= responseMatcher.getValue().equals(onDiskResponseMatcher);

      return !unchanged;
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      message.setResetContextEachRun(resetContextEachRun.toMessage());
      message.setInjectBehaviorState(injectBehaviorState.toMessage());
      message.setInjectEnvironmentState(injectEnvironmentState.toMessage());
      message.setMatchIsSuccess(matchIsSuccess.toMessage());
      message.setSystem(system.toMessage());
      message.setPrompt(prompt.toMessage());
      message.setResponseMatcher(responseMatcher.toMessage());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      resetContextEachRun.fromMessage(message.getResetContextEachRun());
      injectBehaviorState.fromMessage(message.getInjectBehaviorState());
      injectEnvironmentState.fromMessage(message.getInjectEnvironmentState());
      matchIsSuccess.fromMessage(message.getMatchIsSuccess());
      system.fromMessage(message.getSystemAsString());
      prompt.fromMessage(message.getPromptAsString());
      responseMatcher.fromMessage(message.getResponseMatcherAsString());
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

   public CRDTBidirectionalBoolean getMatchIsSuccess()
   {
      return matchIsSuccess;
   }

   public CRDTBidirectionalString getSystem()
   {
      return system;
   }

   public CRDTBidirectionalString getPrompt()
   {
      return prompt;
   }

   public CRDTBidirectionalString getResponseMatcher()
   {
      return responseMatcher;
   }
}

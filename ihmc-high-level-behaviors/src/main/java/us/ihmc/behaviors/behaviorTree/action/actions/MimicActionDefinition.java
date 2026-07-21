package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.MimicActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalString;

public class MimicActionDefinition extends ActionNodeDefinition
{
   public enum MimicActionType
   {
      EXIT_POLICY,
      EXECUTE_POLICY;

      public static final MimicActionType[] values = values();
   }

   private final CRDTBidirectionalEnumField<MimicActionType> mimicActionType;
   private final CRDTBidirectionalString mimicFileName;
   private final CRDTBidirectionalDouble waitTimeExitPolicy;

   private MimicActionType onDiskMimicActionType;
   private String onDiskMimicFileName;
   private double onDiskWaitTimeExitPolicy;

   public MimicActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      mimicActionType = new CRDTBidirectionalEnumField<>(this, MimicActionType.EXECUTE_POLICY);
      mimicFileName = new CRDTBidirectionalString(this, "");
      waitTimeExitPolicy = new CRDTBidirectionalDouble(this, 2.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("mimicActionType", mimicActionType.getValue().name());
      jsonNode.put("mimicFileName", mimicFileName.getValue());
      jsonNode.put("waitTimeExitPolicy", waitTimeExitPolicy.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("mimicActionType"))
      {
         MimicActionType loadedType = MimicActionType.valueOf(jsonNode.get("mimicActionType").textValue());
         mimicActionType.setValue(loadedType);
      }
      else
         mimicActionType.setValue(MimicActionType.EXECUTE_POLICY);

      if (jsonNode.has("mimicFileName"))
         mimicFileName.setValue(jsonNode.get("mimicFileName").textValue());
      else
         mimicFileName.setValue("");

      if (jsonNode.has("waitTimeExitPolicy"))
         waitTimeExitPolicy.setValue(jsonNode.get("waitTimeExitPolicy").asDouble());
      else if (jsonNode.has("transitionDuration")) // backwards compatibility
         waitTimeExitPolicy.setValue(jsonNode.get("transitionDuration").asDouble());
      else if (jsonNode.has("mimicDuration")) // backwards compatibility
         waitTimeExitPolicy.setValue(jsonNode.get("mimicDuration").asDouble());
      else
         waitTimeExitPolicy.setValue(0.0);
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskMimicActionType = mimicActionType.getValue();
      onDiskMimicFileName = mimicFileName.getValue();
      onDiskWaitTimeExitPolicy = waitTimeExitPolicy.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         mimicActionType.setValue(onDiskMimicActionType);
         mimicFileName.setValue(onDiskMimicFileName);
         waitTimeExitPolicy.setValue(onDiskWaitTimeExitPolicy);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= mimicActionType.getValue() == onDiskMimicActionType;
      unchanged &= mimicFileName.getValue().equals(onDiskMimicFileName);
      unchanged &= waitTimeExitPolicy.getValue() == onDiskWaitTimeExitPolicy;

      return !unchanged;
   }

   public void toMessage(MimicActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setMimicActionType(mimicActionType.toMessageOrdinal());
      message.setMimicFileName(mimicFileName.toMessage());
      message.setWaitTimeExitPolicy(waitTimeExitPolicy.toMessage());
   }

   public void fromMessage(MimicActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      mimicActionType.fromMessageOrdinal(message.getMimicActionType(), MimicActionType.values);
      mimicFileName.fromMessage(message.getMimicFileNameAsString());
      waitTimeExitPolicy.fromMessage(message.getWaitTimeExitPolicy());
   }

   public CRDTBidirectionalEnumField<MimicActionType> getMimicActionType()
   {
      return mimicActionType;
   }

   public String getMimicFileName()
   {
      return mimicFileName.getValue();
   }

   public void setMimicFileName(String mimicFileName)
   {
      this.mimicFileName.setValue(mimicFileName);
   }

   public double getWaitTimeExitPolicy()
   {
      return waitTimeExitPolicy.getValue();
   }

   public void setWaitTimeExitPolicy(double waitTimeExitPolicy)
   {
      this.waitTimeExitPolicy.setValue(waitTimeExitPolicy);
   }
}

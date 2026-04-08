package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.MimicActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalString;

public class MimicActionDefinition extends ActionNodeDefinition
{
   public enum MimicActionType
   {
      POLICY_TRANSITION,
      EXIT_POLICY,
      EXECUTE_POLICY;

      public static final MimicActionType[] values = values();
   }

   private final CRDTBidirectionalEnumField<MimicActionType> mimicActionType;
   private final CRDTBidirectionalString mimicFileName;

   private MimicActionType onDiskMimicActionType;
   private String onDiskMimicFileName;

   public MimicActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      mimicActionType = new CRDTBidirectionalEnumField<>(this, MimicActionType.POLICY_TRANSITION);
      mimicFileName = new CRDTBidirectionalString(this, "");
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("mimicActionType", mimicActionType.getValue().name());
      jsonNode.put("mimicFileName", mimicFileName.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("mimicActionType"))
         mimicActionType.setValue(MimicActionType.valueOf(jsonNode.get("mimicActionType").textValue()));
      else
         mimicActionType.setValue(MimicActionType.POLICY_TRANSITION);

      if (jsonNode.has("mimicFileName"))
         mimicFileName.setValue(jsonNode.get("mimicFileName").textValue());
      else
         mimicFileName.setValue("");
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskMimicActionType = mimicActionType.getValue();
      onDiskMimicFileName = mimicFileName.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         mimicActionType.setValue(onDiskMimicActionType);
         mimicFileName.setValue(onDiskMimicFileName);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= mimicActionType.getValue() == onDiskMimicActionType;
      unchanged &= mimicFileName.getValue().equals(onDiskMimicFileName);

      return !unchanged;
   }

   public void toMessage(MimicActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setMimicActionType(mimicActionType.toMessageOrdinal());
      message.setMimicFileName(mimicFileName.toMessage());
   }

   public void fromMessage(MimicActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      mimicActionType.fromMessageOrdinal(message.getMimicActionType(), MimicActionType.values);
      mimicFileName.fromMessage(message.getMimicFileNameAsString());
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
}

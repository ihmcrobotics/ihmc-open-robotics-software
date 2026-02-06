package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

public class BehaviorTreeSceneObjectDefinition extends LatestTimestampModifiable
{
   private final CRDTBidirectionalEnumField<BehaviorTreeSceneObjectType> objectType;
   private final CRDTBidirectionalString yoloModelName;
   private final CRDTBidirectionalString yoloClassName;
   private final CRDTBidirectionalEnumField<IsaacROSFoundationPoseObject> foundationPoseObjectType;

   private BehaviorTreeSceneObjectType onDiskObjectType;
   private String onDiskYoloModelName;
   private String onDiskYoloClassName;
   private IsaacROSFoundationPoseObject onDiskFoundationPoseObjectType;

   /** Used when a scene object state extends this. */
   public BehaviorTreeSceneObjectDefinition(CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(crdtInfo);

      objectType = new CRDTBidirectionalEnumField<>(this, BehaviorTreeSceneObjectType.values[definition.getObjectType()]);
      yoloModelName = new CRDTBidirectionalString(this, definition.getYoloModelNameAsString());
      yoloClassName = new CRDTBidirectionalString(this, definition.getYoloClassNameAsString());
      foundationPoseObjectType = new CRDTBidirectionalEnumField<>(this, IsaacROSFoundationPoseObject.values[definition.getFoundationPoseObjectType()]);
      setModifierName(getName());
   }

   /** Used when this is a field of scene action. The super LatestTimestampModifiable is not used in this case. */
   public BehaviorTreeSceneObjectDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      super(latestTimestampModifiable.getCRDTInfo());

      objectType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, BehaviorTreeSceneObjectType.YOLO_ONLY);
      yoloModelName = new CRDTBidirectionalString(latestTimestampModifiable, "best_multi_01_16_2026");
      yoloClassName = new CRDTBidirectionalString(latestTimestampModifiable, "door_lever");
      foundationPoseObjectType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, IsaacROSFoundationPoseObject.MUSTARD);
      setModifierName(getName());
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("objectType", objectType.getValue().name());
      jsonNode.put("yoloModelName", yoloModelName.getValue());
      if (objectType.getValue() == BehaviorTreeSceneObjectType.YOLO_ONLY)
         jsonNode.put("yoloClassName", yoloClassName.getValue());
      else if (objectType.getValue() == BehaviorTreeSceneObjectType.FOUNDATION_POSE)
         jsonNode.put("foundationPoseObjectType", foundationPoseObjectType.getValue().name());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      objectType.setValue(BehaviorTreeSceneObjectType.valueOf(jsonNode.get("objectType").asText()));
      yoloModelName.setValue(jsonNode.get("yoloModelName").asText());
      if (objectType.getValue() == BehaviorTreeSceneObjectType.YOLO_ONLY)
         yoloClassName.setValue(jsonNode.get("yoloClassName").asText());
      else if (objectType.getValue() == BehaviorTreeSceneObjectType.FOUNDATION_POSE)
         foundationPoseObjectType.setValue(IsaacROSFoundationPoseObject.valueOf(jsonNode.get("foundationPoseObjectType").asText()));
   }

   public void setOnDiskFields()
   {
      onDiskObjectType = objectType.getValue();
      onDiskYoloModelName = yoloModelName.getValue();
      onDiskYoloClassName = yoloClassName.getValue();
      onDiskFoundationPoseObjectType = foundationPoseObjectType.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      if (isUndoAvailable())
      {
         objectType.setValue(onDiskObjectType);
         yoloModelName.setValue(onDiskYoloModelName);
         yoloClassName.setValue(onDiskYoloClassName);
         foundationPoseObjectType.setValue(onDiskFoundationPoseObjectType);
      }
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= objectType.getValue() == onDiskObjectType;
      unchanged &= yoloModelName.getValue().equals(onDiskYoloModelName);
      unchanged &= yoloClassName.getValue().equals(onDiskYoloClassName);
      unchanged &= foundationPoseObjectType.getValue() == onDiskFoundationPoseObjectType;

      return !unchanged;
   }

   public void toMessage(BehaviorTreeSceneObjectDefinitionMessage message)
   {
      message.setObjectType(objectType.toMessageOrdinal());
      message.setYoloModelName(yoloModelName.toMessage());
      message.setYoloClassName(yoloClassName.toMessage());
      message.setFoundationPoseObjectType(foundationPoseObjectType.toMessageOrdinal());
   }

   public void fromMessage(BehaviorTreeSceneObjectDefinitionMessage message)
   {
      objectType.fromMessageOrdinal(message.getObjectType(), BehaviorTreeSceneObjectType.values());
      yoloModelName.fromMessage(message.getYoloModelNameAsString());
      yoloClassName.fromMessage(message.getYoloClassNameAsString());
      foundationPoseObjectType.fromMessageOrdinal(message.getFoundationPoseObjectType(), IsaacROSFoundationPoseObject.values);
   }

   public String getName()
   {
      return switch (objectType.getValue())
      {
         case YOLO_ONLY -> yoloClassName.getValue();
         case FOUNDATION_POSE -> foundationPoseObjectType.getValue().titleCaseName;
         case DOOR_PANEL -> "Door Panel";
      };
   }

   public BehaviorTreeSceneObjectType getObjectType()
   {
      return objectType.getValue();
   }

   public void setObjectType(BehaviorTreeSceneObjectType objectType)
   {
      this.objectType.setValue(objectType);
   }

   public String getYoloModelName()
   {
      return yoloModelName.getValue();
   }

   public void setYoloModelName(String yoloModelName)
   {
      this.yoloModelName.setValue(yoloModelName);
   }

   public String getYoloClassName()
   {
      return yoloClassName.getValue();
   }

   public void setYoloClassName(String yoloClassName)
   {
      this.yoloClassName.setValue(yoloClassName);
   }

   public IsaacROSFoundationPoseObject getFoundationPoseObjectType()
   {
      return foundationPoseObjectType.getValue();
   }

   public void setFoundationPoseObjectType(IsaacROSFoundationPoseObject foundationPoseObjectType)
   {
      this.foundationPoseObjectType.setValue(foundationPoseObjectType);
   }

   protected boolean isUndoAvailable()
   {
      return onDiskObjectType != null;
   }
}

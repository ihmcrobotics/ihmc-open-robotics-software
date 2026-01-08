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

   public BehaviorTreeSceneObjectDefinition(CRDTInfo crdtInfo)
   {
      super(crdtInfo);

      objectType = new CRDTBidirectionalEnumField<>(this, BehaviorTreeSceneObjectType.YOLO_ONLY);
      yoloModelName = new CRDTBidirectionalString(this, "best_multi_12_17_2025");
      yoloClassName = new CRDTBidirectionalString(this, "door_lever");
      foundationPoseObjectType = new CRDTBidirectionalEnumField<>(this, IsaacROSFoundationPoseObject.MUSTARD);
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      if (objectType.getValue() != null)
         jsonNode.put("objectType", objectType.getValue().name());
      jsonNode.put("yoloModelName", yoloModelName.getValue());
      jsonNode.put("yoloClassName", yoloClassName.getValue());
      jsonNode.put("foundationPoseObjectType", foundationPoseObjectType.getValue().name());
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      if (jsonNode.has("objectType"))
         objectType.setValue(BehaviorTreeSceneObjectType.valueOf(jsonNode.get("objectType").asText()));
      yoloModelName.setValue(jsonNode.get("yoloModelName").asText());
      yoloClassName.setValue(jsonNode.get("yoloClassName").asText());
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
      return objectType.getValue() == BehaviorTreeSceneObjectType.YOLO_ONLY ? yoloClassName.getValue() : foundationPoseObjectType.getValue().titleCaseName;
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

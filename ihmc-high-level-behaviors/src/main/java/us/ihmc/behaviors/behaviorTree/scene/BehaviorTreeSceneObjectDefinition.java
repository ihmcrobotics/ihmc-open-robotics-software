package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
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
   private final CRDTBidirectionalInteger minPostPoints;
   private final CRDTBidirectionalInteger minRecessPoints;
   private final CRDTBidirectionalString customFrameName;
   private final CRDTBidirectionalString frameA;
   private final CRDTBidirectionalString frameB;
   private final CRDTBidirectionalFloat distance;

   private BehaviorTreeSceneObjectType onDiskObjectType;
   private String onDiskYoloModelName;
   private String onDiskYoloClassName;
   private IsaacROSFoundationPoseObject onDiskFoundationPoseObjectType;
   private int onDiskMinPostPoints;
   private int onDiskMinRecessPoints;
   private String onDiskCustomFrameName;
   private String onDiskFrameA;
   private String onDiskFrameB;
   private float onDiskDistance;

   /** Used when a scene object state extends this. */
   public BehaviorTreeSceneObjectDefinition(CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(crdtInfo);

      objectType = new CRDTBidirectionalEnumField<>(this, BehaviorTreeSceneObjectType.values[definition.getObjectType()]);
      yoloModelName = new CRDTBidirectionalString(this, definition.getYoloModelNameAsString());
      yoloClassName = new CRDTBidirectionalString(this, definition.getYoloClassNameAsString());
      foundationPoseObjectType = new CRDTBidirectionalEnumField<>(this, IsaacROSFoundationPoseObject.values[definition.getFoundationPoseObjectType()]);
      minPostPoints = new CRDTBidirectionalInteger(this, definition.getMinPostPoints());
      minRecessPoints = new CRDTBidirectionalInteger(this, definition.getMinRecessPoints());
      customFrameName = new CRDTBidirectionalString(this, definition.getCustomFrameNameAsString());
      frameA = new CRDTBidirectionalString(this, definition.getFrameAAsString());
      frameB = new CRDTBidirectionalString(this, definition.getFrameBAsString());
      distance = new CRDTBidirectionalFloat(this, definition.getDistance());
      setModifierName(getName());
   }

   /** Used when this is a field of scene action. The super LatestTimestampModifiable is not used in this case. */
   public BehaviorTreeSceneObjectDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      super(latestTimestampModifiable.getCRDTInfo());

      objectType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, BehaviorTreeSceneObjectType.YOLO_ONLY);
      yoloModelName = new CRDTBidirectionalString(latestTimestampModifiable, "best_multi_02_17_2026");
      yoloClassName = new CRDTBidirectionalString(latestTimestampModifiable, "door_lever");
      foundationPoseObjectType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, IsaacROSFoundationPoseObject.MUSTARD);
      minPostPoints = new CRDTBidirectionalInteger(latestTimestampModifiable, 1000);
      minRecessPoints = new CRDTBidirectionalInteger(latestTimestampModifiable, 3000);
      customFrameName = new CRDTBidirectionalString(latestTimestampModifiable, "");
      frameA = new CRDTBidirectionalString(latestTimestampModifiable, "");
      frameB = new CRDTBidirectionalString(latestTimestampModifiable, "");
      distance = new CRDTBidirectionalFloat(latestTimestampModifiable, 0.0f);
      setModifierName(getName());
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("objectType", objectType.getValue().name());

      switch (objectType.getValue())
      {
         case YOLO_ONLY ->
         {
            jsonNode.put("yoloModelName", yoloModelName.getValue());
            if (objectType.getValue() == BehaviorTreeSceneObjectType.YOLO_ONLY)
               jsonNode.put("yoloClassName", yoloClassName.getValue());
         }
         case FOUNDATION_POSE -> jsonNode.put("foundationPoseObjectType", foundationPoseObjectType.getValue().name());
         case DOOR_FRAME ->
         {
            jsonNode.put("minPostPoints", minPostPoints.getValue());
            jsonNode.put("minRecessPoints", minRecessPoints.getValue());
         }
         case CUSTOM_FRAME ->
         {
            jsonNode.put("customFrameName", customFrameName.getValue());
            jsonNode.put("frameA", frameA.getValue());
            jsonNode.put("frameB", frameB.getValue());
            jsonNode.put("distance", distance.getValue());
         }
      }
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      objectType.setValue(BehaviorTreeSceneObjectType.valueOf(jsonNode.get("objectType").asText()));

      switch (objectType.getValue())
      {
         case YOLO_ONLY ->
         {
            yoloModelName.setValue(jsonNode.get("yoloModelName").asText());
            if (objectType.getValue() == BehaviorTreeSceneObjectType.YOLO_ONLY)
               yoloClassName.setValue(jsonNode.get("yoloClassName").asText());
         }
         case FOUNDATION_POSE -> foundationPoseObjectType.setValue(IsaacROSFoundationPoseObject.valueOf(jsonNode.get("foundationPoseObjectType").asText()));
         case DOOR_FRAME ->
         {
            minPostPoints.setValue(jsonNode.has("minPostPoints") ? jsonNode.get("minPostPoints").asInt() : 400);
            minRecessPoints.setValue(jsonNode.has("minRecessPoints") ? jsonNode.get("minRecessPoints").asInt() : 3000);
         }
         case CUSTOM_FRAME ->
         {
            customFrameName.setValue(jsonNode.has("customFrameName") ? jsonNode.get("customFrameName").asText() : "");
            frameA.setValue(jsonNode.has("frameA") ? jsonNode.get("frameA").asText() : "");
            frameB.setValue(jsonNode.has("frameB") ? jsonNode.get("frameB").asText() : "");
            distance.setValue(jsonNode.has("distance") ? (float) jsonNode.get("distance").asDouble() : 0.0f);
         }
      }
   }

   public void setOnDiskFields()
   {
      onDiskObjectType = objectType.getValue();
      onDiskYoloModelName = yoloModelName.getValue();
      onDiskYoloClassName = yoloClassName.getValue();
      onDiskFoundationPoseObjectType = foundationPoseObjectType.getValue();
      onDiskMinPostPoints = minPostPoints.getValue();
      onDiskMinRecessPoints = minRecessPoints.getValue();
      onDiskCustomFrameName = customFrameName.getValue();
      onDiskFrameA = frameA.getValue();
      onDiskFrameB = frameB.getValue();
      onDiskDistance = distance.getValue();
   }

   public void undoAllNontopologicalChanges()
   {
      if (isUndoAvailable())
      {
         objectType.setValue(onDiskObjectType);
         yoloModelName.setValue(onDiskYoloModelName);
         yoloClassName.setValue(onDiskYoloClassName);
         foundationPoseObjectType.setValue(onDiskFoundationPoseObjectType);
         minPostPoints.setValue(onDiskMinPostPoints);
         minRecessPoints.setValue(onDiskMinRecessPoints);
         customFrameName.setValue(onDiskCustomFrameName);
         frameA.setValue(onDiskFrameA);
         frameB.setValue(onDiskFrameB);
         distance.setValue(onDiskDistance);
      }
   }

   public boolean hasChanges()
   {
      boolean unchanged = true;

      unchanged &= objectType.getValue() == onDiskObjectType;
      unchanged &= yoloModelName.getValue().equals(onDiskYoloModelName);
      unchanged &= yoloClassName.getValue().equals(onDiskYoloClassName);
      unchanged &= foundationPoseObjectType.getValue() == onDiskFoundationPoseObjectType;
      unchanged &= minPostPoints.getValue() == onDiskMinPostPoints;
      unchanged &= minRecessPoints.getValue() == onDiskMinRecessPoints;
      unchanged &= customFrameName.getValue().equals(onDiskCustomFrameName);
      unchanged &= frameA.getValue().equals(onDiskFrameA);
      unchanged &= frameB.getValue().equals(onDiskFrameB);
      unchanged &= distance.getValue() == onDiskDistance;

      return !unchanged;
   }

   public void toMessage(BehaviorTreeSceneObjectDefinitionMessage message)
   {
      message.setObjectType(objectType.toMessageOrdinal());
      message.setYoloModelName(yoloModelName.toMessage());
      message.setYoloClassName(yoloClassName.toMessage());
      message.setFoundationPoseObjectType(foundationPoseObjectType.toMessageOrdinal());
      message.setMinPostPoints(minPostPoints.toMessage());
      message.setMinRecessPoints(minRecessPoints.toMessage());
      message.setCustomFrameName(customFrameName.toMessage());
      message.setFrameA(frameA.toMessage());
      message.setFrameB(frameB.toMessage());
      message.setDistance(distance.toMessage());
   }

   public void fromMessage(BehaviorTreeSceneObjectDefinitionMessage message)
   {
      objectType.fromMessageOrdinal(message.getObjectType(), BehaviorTreeSceneObjectType.values());
      yoloModelName.fromMessage(message.getYoloModelNameAsString());
      yoloClassName.fromMessage(message.getYoloClassNameAsString());
      foundationPoseObjectType.fromMessageOrdinal(message.getFoundationPoseObjectType(), IsaacROSFoundationPoseObject.values);
      minPostPoints.fromMessage(message.getMinPostPoints());
      minRecessPoints.fromMessage(message.getMinRecessPoints());
      customFrameName.fromMessage(message.getCustomFrameNameAsString());
      frameA.fromMessage(message.getFrameAAsString());
      frameB.fromMessage(message.getFrameBAsString());
      distance.fromMessage(message.getDistance());
   }

   public String getName()
   {
      return switch (objectType.getValue())
      {
         case YOLO_ONLY -> yoloClassName.getValue();
         case CUSTOM_FRAME -> customFrameName.getValue();
         case FOUNDATION_POSE -> foundationPoseObjectType.getValue().titleCaseName;
         case DOOR_PANEL -> "Door Panel";
         case DOOR_FRAME -> "Door Frame";
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

   public int getMinPostPoints()
   {
      return minPostPoints.getValue();
   }

   public void setMinPostPoints(int minPostPoints)
   {
      this.minPostPoints.setValue(minPostPoints);
   }

   public int getMinRecessPoints()
   {
      return minRecessPoints.getValue();
   }

   public void setMinRecessPoints(int minRecessPoints)
   {
      this.minRecessPoints.setValue(minRecessPoints);
   }

   public String getCustomFrameName()
   {
      return customFrameName.getValue();
   }

   public void setCustomFrameName(String customFrameName)
   {
      this.customFrameName.setValue(customFrameName);
   }

   public String getFrameA()
   {
      return frameA.getValue();
   }

   public void setFrameA(String frameA)
   {
      this.frameA.setValue(frameA);
   }

   public String getFrameB()
   {
      return frameB.getValue();
   }

   public void setFrameB(String frameB)
   {
      this.frameB.setValue(frameB);
   }

   public float getDistance()
   {
      return distance.getValue();
   }

   public void setDistance(float distance)
   {
      this.distance.setValue(distance);
   }

   protected boolean isUndoAvailable()
   {
      return onDiskObjectType != null;
   }
}

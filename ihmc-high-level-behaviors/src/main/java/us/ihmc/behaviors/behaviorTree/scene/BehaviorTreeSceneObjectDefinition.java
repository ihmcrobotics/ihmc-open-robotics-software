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
   public enum CompositeFrameType
   {
      APPROACH,
      HYBRID;

      public static final CompositeFrameType[] values = values();
   }

   private final CRDTBidirectionalEnumField<BehaviorTreeSceneObjectType> objectType;
   private final CRDTBidirectionalString yoloModelName;
   private final CRDTBidirectionalString yoloClassName;
   private final CRDTBidirectionalEnumField<IsaacROSFoundationPoseObject> foundationPoseObjectType;
   private final CRDTBidirectionalInteger minPostPoints;
   private final CRDTBidirectionalInteger minRecessPoints;
   private final CRDTBidirectionalInteger minCapsulePoints;
   private final CRDTBidirectionalFloat searchStartX;
   private final CRDTBidirectionalString compositeFrameName;
   private final CRDTBidirectionalString compositeFrameA;
   private final CRDTBidirectionalString compositeFrameB;
   private final CRDTBidirectionalEnumField<CompositeFrameType> compositeFrameType;
   private final CRDTBidirectionalFloat compositeDistance;

   private BehaviorTreeSceneObjectType onDiskObjectType;
   private String onDiskYoloModelName;
   private String onDiskYoloClassName;
   private IsaacROSFoundationPoseObject onDiskFoundationPoseObjectType;
   private int onDiskMinPostPoints;
   private int onDiskMinRecessPoints;
   private int onDiskMinCapsulePoints;
   private float onDiskSearchStartX;
   private String onDiskCompositeFrameName;
   private String onDiskCompositeFrameA;
   private String onDiskCompositeFrameB;
   private CompositeFrameType onDiskCompositeFrameType;
   private float onDiskCompositeDistance;

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
      minCapsulePoints = new CRDTBidirectionalInteger(this, definition.getMinCapsulePoints());
      searchStartX = new CRDTBidirectionalFloat(this, definition.getSearchStartX());
      compositeFrameName = new CRDTBidirectionalString(this, definition.getCompositeFrameNameAsString());
      compositeFrameA = new CRDTBidirectionalString(this, definition.getCompositeFrameAAsString());
      compositeFrameB = new CRDTBidirectionalString(this, definition.getCompositeFrameBAsString());
      compositeFrameType = new CRDTBidirectionalEnumField<>(this, compositeFrameTypeFromOrdinal(definition.getCompositeFrameType()));
      compositeDistance = new CRDTBidirectionalFloat(this, definition.getCompositeDistance());
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
      minCapsulePoints = new CRDTBidirectionalInteger(latestTimestampModifiable, 300);
      searchStartX = new CRDTBidirectionalFloat(latestTimestampModifiable, 0.05f);
      compositeFrameName = new CRDTBidirectionalString(latestTimestampModifiable, "");
      compositeFrameA = new CRDTBidirectionalString(latestTimestampModifiable, "");
      compositeFrameB = new CRDTBidirectionalString(latestTimestampModifiable, "");
      compositeFrameType = new CRDTBidirectionalEnumField<>(latestTimestampModifiable, CompositeFrameType.APPROACH);
      compositeDistance = new CRDTBidirectionalFloat(latestTimestampModifiable, 0.0f);
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
         case COMPOSITE_FRAME ->
         {
            jsonNode.put("compositeFrameName", compositeFrameName.getValue());
            jsonNode.put("compositeFrameA", compositeFrameA.getValue());
            jsonNode.put("compositeFrameB", compositeFrameB.getValue());
            jsonNode.put("compositeFrameType", compositeFrameType.getValue().name());
            if (compositeFrameType.getValue() == CompositeFrameType.APPROACH)
               jsonNode.put("compositeDistance", compositeDistance.getValue());
         }
         case DOOR_PANEL ->
         {
         }
         case DOOR_FRAME ->
         {
            jsonNode.put("minPostPoints", minPostPoints.getValue());
            jsonNode.put("minRecessPoints", minRecessPoints.getValue());
         }
         case APPROACH_TABLE ->
         {
            jsonNode.put("minCapsulePoints", minCapsulePoints.getValue());
            jsonNode.put("searchStartX", searchStartX.getValue());
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
         case COMPOSITE_FRAME ->
         {
            compositeFrameName.setValue(jsonNode.has("compositeFrameName") ? jsonNode.get("compositeFrameName").asText() : "");
            compositeFrameA.setValue(jsonNode.has("compositeFrameA") ? jsonNode.get("compositeFrameA").asText() : "");
            compositeFrameB.setValue(jsonNode.has("compositeFrameB") ? jsonNode.get("compositeFrameB").asText() : "");
            if (jsonNode.has("compositeFrameType"))
               compositeFrameType.setValue(CompositeFrameType.valueOf(jsonNode.get("compositeFrameType").asText()));
            if (compositeFrameType.getValue() == CompositeFrameType.APPROACH)
               compositeDistance.setValue(jsonNode.has("compositeDistance") ? (float) jsonNode.get("compositeDistance").asDouble() : 0.0f);
         }
         case DOOR_PANEL ->
         {
         }
         case DOOR_FRAME ->
         {
            minPostPoints.setValue(jsonNode.has("minPostPoints") ? jsonNode.get("minPostPoints").asInt() : 400);
            minRecessPoints.setValue(jsonNode.has("minRecessPoints") ? jsonNode.get("minRecessPoints").asInt() : 3000);
         }
         case APPROACH_TABLE ->
         {
            minCapsulePoints.setValue(jsonNode.has("minCapsulePoints") ? jsonNode.get("minCapsulePoints").asInt() : 300);
            searchStartX.setValue(jsonNode.has("searchStartX") ? (float) jsonNode.get("searchStartX").asDouble() : 0.05f);
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
      onDiskMinCapsulePoints = minCapsulePoints.getValue();
      onDiskSearchStartX = searchStartX.getValue();
      onDiskCompositeFrameName = compositeFrameName.getValue();
      onDiskCompositeFrameA = compositeFrameA.getValue();
      onDiskCompositeFrameB = compositeFrameB.getValue();
      onDiskCompositeFrameType = compositeFrameType.getValue();
      onDiskCompositeDistance = compositeDistance.getValue();
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
         minCapsulePoints.setValue(onDiskMinCapsulePoints);
         searchStartX.setValue(onDiskSearchStartX);
         compositeFrameName.setValue(onDiskCompositeFrameName);
         compositeFrameA.setValue(onDiskCompositeFrameA);
         compositeFrameB.setValue(onDiskCompositeFrameB);
         compositeFrameType.setValue(onDiskCompositeFrameType);
         compositeDistance.setValue(onDiskCompositeDistance);
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
      unchanged &= minCapsulePoints.getValue() == onDiskMinCapsulePoints;
      unchanged &= searchStartX.getValue() == onDiskSearchStartX;
      unchanged &= compositeFrameName.getValue().equals(onDiskCompositeFrameName);
      unchanged &= compositeFrameA.getValue().equals(onDiskCompositeFrameA);
      unchanged &= compositeFrameB.getValue().equals(onDiskCompositeFrameB);
      unchanged &= compositeFrameType.getValue() == onDiskCompositeFrameType;
      unchanged &= compositeDistance.getValue() == onDiskCompositeDistance;

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
      message.setMinCapsulePoints(minCapsulePoints.toMessage());
      message.setSearchStartX(searchStartX.toMessage());
      message.setCompositeFrameName(compositeFrameName.toMessage());
      message.setCompositeFrameA(compositeFrameA.toMessage());
      message.setCompositeFrameB(compositeFrameB.toMessage());
      message.setCompositeFrameType(compositeFrameType.toMessageOrdinal());
      message.setCompositeDistance(compositeDistance.toMessage());
   }

   public void fromMessage(BehaviorTreeSceneObjectDefinitionMessage message)
   {
      objectType.fromMessageOrdinal(message.getObjectType(), BehaviorTreeSceneObjectType.values());
      yoloModelName.fromMessage(message.getYoloModelNameAsString());
      yoloClassName.fromMessage(message.getYoloClassNameAsString());
      foundationPoseObjectType.fromMessageOrdinal(message.getFoundationPoseObjectType(), IsaacROSFoundationPoseObject.values);
      minPostPoints.fromMessage(message.getMinPostPoints());
      minRecessPoints.fromMessage(message.getMinRecessPoints());
      minCapsulePoints.fromMessage(message.getMinCapsulePoints());
      searchStartX.fromMessage(message.getSearchStartX());
      compositeFrameName.fromMessage(message.getCompositeFrameNameAsString());
      compositeFrameA.fromMessage(message.getCompositeFrameAAsString());
      compositeFrameB.fromMessage(message.getCompositeFrameBAsString());
      compositeFrameType.fromMessageOrdinal(message.getCompositeFrameType(), CompositeFrameType.values);
      compositeDistance.fromMessage(message.getCompositeDistance());
   }

   public String getName()
   {
      return switch (objectType.getValue())
      {
         case YOLO_ONLY -> yoloClassName.getValue();
         case FOUNDATION_POSE -> foundationPoseObjectType.getValue().titleCaseName;
         case COMPOSITE_FRAME -> compositeFrameName.getValue();
         case DOOR_PANEL -> "Door Panel";
         case DOOR_FRAME -> "Door Frame";
         case APPROACH_TABLE -> "Approach Table";
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

   public int getMinCapsulePoints()
   {
      return minCapsulePoints.getValue();
   }

   public void setMinCapsulePoints(int minCapsulePoints)
   {
      this.minCapsulePoints.setValue(minCapsulePoints);
   }

   public float getSearchStartX()
   {
      return searchStartX.getValue();
   }

   public void setSearchStartX(float searchStartX)
   {
      this.searchStartX.setValue(searchStartX);
   }

   public String getCompositeFrameName()
   {
      return compositeFrameName.getValue();
   }

   public void setCompositeFrameName(String compositeFrameName)
   {
      this.compositeFrameName.setValue(compositeFrameName);
   }

   public String getCompositeFrameA()
   {
      return compositeFrameA.getValue();
   }

   public void setCompositeFrameA(String compositeFrameA)
   {
      this.compositeFrameA.setValue(compositeFrameA);
   }

   public String getCompositeFrameB()
   {
      return compositeFrameB.getValue();
   }

   public void setCompositeFrameB(String compositeFrameB)
   {
      this.compositeFrameB.setValue(compositeFrameB);
   }

   public CompositeFrameType getCompositeFrameType()
   {
      return compositeFrameType.getValue();
   }

   public void setCompositeFrameType(CompositeFrameType compositeFrameType)
   {
      this.compositeFrameType.setValue(compositeFrameType);
   }

   public float getCompositeDistance()
   {
      return compositeDistance.getValue();
   }

   public void setCompositeDistance(float compositeDistance)
   {
      this.compositeDistance.setValue(compositeDistance);
   }

   protected boolean isUndoAvailable()
   {
      return onDiskObjectType != null;
   }

   private static CompositeFrameType compositeFrameTypeFromOrdinal(byte ordinal)
   {
      return ordinal >= 0 && ordinal < CompositeFrameType.values.length ? CompositeFrameType.values[ordinal] : CompositeFrameType.APPROACH;
   }
}

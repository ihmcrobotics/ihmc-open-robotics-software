package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumList;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerList;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.CRDTBidirectionalStringList;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

import java.util.ArrayList;
import java.util.List;

public class SceneActionNodeDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalString yoloModelName;
   private final CRDTBidirectionalFloat yoloConfidenceThreshold;
   private final CRDTBidirectionalFloat yoloMaskThreshold;
   private final CRDTBidirectionalInteger segmentationMaskErosionRadius;
   private final CRDTBidirectionalFloat outlierThreshold;
   private final CRDTBidirectionalEnumField<IsaacROSFoundationPoseObject> objectType;
   private final CRDTBidirectionalBoolean useFoundationPose;
   private final CRDTBidirectionalStringList enabledYoloModels;
   private final CRDTBidirectionalIntegerList ignoredYoloClassIndices;
   private final CRDTBidirectionalEnumList<IsaacROSFoundationPoseObject> enabledFoundationPoseModels;

   private String onDiskYoloModelName;
   private float onDiskYoloConfidenceThreshold;
   private float onDiskYoloMaskThreshold;
   private int onDiskSegmentationMaskErosionRadius;
   private float onDiskOutlierThreshold;
   private IsaacROSFoundationPoseObject onDiskObjectType;
   private boolean onDiskUseFoundationPose;
   private final List<String> onDiskEnabledYoloModels = new ArrayList<>();
   private final TIntArrayList onDiskIgnoredYoloClassIndices = new TIntArrayList();
   private final List<IsaacROSFoundationPoseObject> onDiskEnabledFoundationPoseModels = new ArrayList<>();

   public SceneActionNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      yoloModelName = new CRDTBidirectionalString(this, "");
      yoloConfidenceThreshold = new CRDTBidirectionalFloat(this, 0.7f);
      yoloMaskThreshold = new CRDTBidirectionalFloat(this, 0.0f);
      segmentationMaskErosionRadius = new CRDTBidirectionalInteger(this, 1);
      outlierThreshold = new CRDTBidirectionalFloat(this, 1.0f);
      objectType = new CRDTBidirectionalEnumField<>(this, IsaacROSFoundationPoseObject.MUSTARD);
      useFoundationPose = new CRDTBidirectionalBoolean(this, false);
      enabledYoloModels = new CRDTBidirectionalStringList(this);
      ignoredYoloClassIndices = new CRDTBidirectionalIntegerList(this);
      enabledFoundationPoseModels = new CRDTBidirectionalEnumList<>(this);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("yoloModelName", yoloModelName.getValue());
      jsonNode.put("yoloConfidenceThreshold", yoloConfidenceThreshold.getValue());
      jsonNode.put("yoloMaskThreshold", yoloMaskThreshold.getValue());
      jsonNode.put("segmentationMaskErosionRadius", segmentationMaskErosionRadius.getValue());
      jsonNode.put("outlierThreshold", outlierThreshold.getValue());
      jsonNode.put("objectType", objectType.getValue().name());
      jsonNode.put("useFoundationPose", useFoundationPose.getValue());

      ArrayNode enabledYoloModelsArray = jsonNode.putArray("enabledYoloModels");
      for (int i = 0; i < enabledYoloModels.getSize(); i++)
         enabledYoloModelsArray.add(enabledYoloModels.getValueReadOnly(i));

      ArrayNode ignoredYoloClassesArray = jsonNode.putArray("ignoredYoloClassIndices");
      for (int i = 0; i < ignoredYoloClassIndices.getSize(); i++)
         ignoredYoloClassesArray.add(ignoredYoloClassIndices.getValueReadOnly(i));

      ArrayNode enabledFoundationPoseModelsArray = jsonNode.putArray("enabledFoundationPoseModels");
      for (int i = 0; i < enabledFoundationPoseModels.getSize(); i++)
         enabledFoundationPoseModelsArray.add(enabledFoundationPoseModels.getValueReadOnly(i).name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("yoloModelName"))
         yoloModelName.setValue(jsonNode.get("yoloModelName").asText());
      if (jsonNode.has("yoloConfidenceThreshold"))
         yoloConfidenceThreshold.setValue((float) jsonNode.get("yoloConfidenceThreshold").asDouble());
      if (jsonNode.has("yoloMaskThreshold"))
         yoloMaskThreshold.setValue((float) jsonNode.get("yoloMaskThreshold").asDouble());
      if (jsonNode.has("segmentationMaskErosionRadius"))
         segmentationMaskErosionRadius.setValue(jsonNode.get("segmentationMaskErosionRadius").asInt());
      if (jsonNode.has("outlierThreshold"))
         outlierThreshold.setValue((float) jsonNode.get("outlierThreshold").asDouble());
      if (jsonNode.has("objectType"))
         objectType.setValue(IsaacROSFoundationPoseObject.valueOf(jsonNode.get("objectType").asText()));
      if (jsonNode.has("useFoundationPose"))
         useFoundationPose.setValue(jsonNode.get("useFoundationPose").asBoolean());

      if (jsonNode.has("enabledYoloModels"))
      {
         ArrayNode enabledYoloModelsArray = (ArrayNode) jsonNode.get("enabledYoloModels");
         enabledYoloModels.clear();
         for (int i = 0; i < enabledYoloModelsArray.size(); i++)
            enabledYoloModels.setValue(i, enabledYoloModelsArray.get(i).asText());
      }

      if (jsonNode.has("ignoredYoloClassIndices"))
      {
         ArrayNode ignoredYoloClassesArray = (ArrayNode) jsonNode.get("ignoredYoloClassIndices");
         ignoredYoloClassIndices.clear();
         for (int i = 0; i < ignoredYoloClassesArray.size(); i++)
            ignoredYoloClassIndices.setValue(i, ignoredYoloClassesArray.get(i).asInt());
      }

      if (jsonNode.has("enabledFoundationPoseModels"))
      {
         ArrayNode enabledFoundationPoseModelsArray = (ArrayNode) jsonNode.get("enabledFoundationPoseModels");
         enabledFoundationPoseModels.clear();
         for (int i = 0; i < enabledFoundationPoseModelsArray.size(); i++)
            enabledFoundationPoseModels.setValue(i, IsaacROSFoundationPoseObject.valueOf(enabledFoundationPoseModelsArray.get(i).asText()));
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskYoloModelName = yoloModelName.getValue();
      onDiskYoloConfidenceThreshold = yoloConfidenceThreshold.getValue();
      onDiskYoloMaskThreshold = yoloMaskThreshold.getValue();
      onDiskSegmentationMaskErosionRadius = segmentationMaskErosionRadius.getValue();
      onDiskOutlierThreshold = outlierThreshold.getValue();
      onDiskObjectType = objectType.getValue();
      onDiskUseFoundationPose = useFoundationPose.getValue();

      onDiskEnabledYoloModels.clear();
      onDiskEnabledYoloModels.addAll(enabledYoloModels.getValue());

      onDiskIgnoredYoloClassIndices.clear();
      onDiskIgnoredYoloClassIndices.addAll(ignoredYoloClassIndices.getValue());

      onDiskEnabledFoundationPoseModels.clear();
      onDiskEnabledFoundationPoseModels.addAll(enabledFoundationPoseModels.getValue());
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         yoloModelName.setValue(onDiskYoloModelName);
         yoloConfidenceThreshold.setValue(onDiskYoloConfidenceThreshold);
         yoloMaskThreshold.setValue(onDiskYoloMaskThreshold);
         segmentationMaskErosionRadius.setValue(onDiskSegmentationMaskErosionRadius);
         outlierThreshold.setValue(onDiskOutlierThreshold);
         objectType.setValue(onDiskObjectType);
         useFoundationPose.setValue(onDiskUseFoundationPose);

         enabledYoloModels.clear();
         enabledYoloModels.getValue().addAll(onDiskEnabledYoloModels);

         ignoredYoloClassIndices.clear();
         ignoredYoloClassIndices.getValue().addAll(onDiskIgnoredYoloClassIndices);

         enabledFoundationPoseModels.clear();
         enabledFoundationPoseModels.getValue().addAll(onDiskEnabledFoundationPoseModels);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= yoloModelName.getValue().equals(onDiskYoloModelName);
      unchanged &= yoloConfidenceThreshold.getValue() == onDiskYoloConfidenceThreshold;
      unchanged &= yoloMaskThreshold.getValue() == onDiskYoloMaskThreshold;
      unchanged &= segmentationMaskErosionRadius.getValue() == onDiskSegmentationMaskErosionRadius;
      unchanged &= outlierThreshold.getValue() == onDiskOutlierThreshold;
      unchanged &= objectType.getValue() == onDiskObjectType;
      unchanged &= useFoundationPose.getValue() == onDiskUseFoundationPose;

      unchanged &= enabledYoloModels.getSize() == onDiskEnabledYoloModels.size();
      for (int i = 0; unchanged && i < enabledYoloModels.getSize(); i++)
         unchanged = enabledYoloModels.getValueReadOnly(i).equals(onDiskEnabledYoloModels.get(i));

      unchanged &= ignoredYoloClassIndices.getSize() == onDiskIgnoredYoloClassIndices.size();
      for (int i = 0; unchanged && i < ignoredYoloClassIndices.getSize(); i++)
         unchanged = ignoredYoloClassIndices.getValueReadOnly(i) == onDiskIgnoredYoloClassIndices.get(i);

      unchanged &= enabledFoundationPoseModels.getSize() == onDiskEnabledFoundationPoseModels.size();
      for (int i = 0; unchanged && i < enabledFoundationPoseModels.getSize(); i++)
         unchanged = enabledFoundationPoseModels.getValueReadOnly(i).equals(onDiskEnabledFoundationPoseModels.get(i));

      return !unchanged;
   }

   public void toMessage(SceneActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setYoloModelName(yoloModelName.getValue());
      message.setYoloConfidenceThreshold(yoloConfidenceThreshold.getValue());
      message.setYoloMaskThreshold(yoloMaskThreshold.getValue());
      message.setSegmentationMaskErosionRadius(segmentationMaskErosionRadius.getValue());
      message.setOutlierThreshold(outlierThreshold.getValue());
      message.setObjectType(objectType.toMessageOrdinal());
      message.setUseFoundationPose(useFoundationPose.getValue());
      enabledYoloModels.toMessage(message.getEnabledYoloModels());
      ignoredYoloClassIndices.toMessage(message.getIgnoredYoloClassIndices());
      enabledFoundationPoseModels.toMessage(message.getEnabledFoundationPoseModels());
   }

   public void fromMessage(SceneActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      yoloModelName.setValue(message.getYoloModelNameAsString());
      yoloConfidenceThreshold.setValue(message.getYoloConfidenceThreshold());
      yoloMaskThreshold.setValue(message.getYoloMaskThreshold());
      segmentationMaskErosionRadius.setValue(message.getSegmentationMaskErosionRadius());
      outlierThreshold.setValue(message.getOutlierThreshold());
      objectType.fromMessageOrdinal(message.getObjectType(), IsaacROSFoundationPoseObject.values);
      useFoundationPose.setValue(message.getUseFoundationPose());
      enabledYoloModels.fromMessage(message.getEnabledYoloModels());
      ignoredYoloClassIndices.fromMessage(message.getIgnoredYoloClassIndices());
      enabledFoundationPoseModels.fromMessage(message.getEnabledFoundationPoseModels(), IsaacROSFoundationPoseObject.values);
   }

   public String getYoloModelName()
   {
      return yoloModelName.getValue();
   }

   public void setYoloModelName(String yoloModelName)
   {
      this.yoloModelName.setValue(yoloModelName);
   }

   public double getYoloConfidenceThreshold()
   {
      return yoloConfidenceThreshold.getValue();
   }

   public void setYoloConfidenceThreshold(float yoloConfidenceThreshold)
   {
      this.yoloConfidenceThreshold.setValue(yoloConfidenceThreshold);
   }

   public double getYoloMaskThreshold()
   {
      return yoloMaskThreshold.getValue();
   }

   public void setYoloMaskThreshold(float yoloMaskThreshold)
   {
      this.yoloMaskThreshold.setValue(yoloMaskThreshold);
   }

   public int getSegmentationMaskErosionRadius()
   {
      return segmentationMaskErosionRadius.getValue();
   }

   public void setSegmentationMaskErosionRadius(int segmentationMaskErosionRadius)
   {
      this.segmentationMaskErosionRadius.setValue(segmentationMaskErosionRadius);
   }

   public float getOutlierThreshold()
   {
      return outlierThreshold.getValue();
   }

   public void setOutlierThreshold(float outlierThreshold)
   {
      this.outlierThreshold.setValue(outlierThreshold);
   }

   public IsaacROSFoundationPoseObject getObjectType()
   {
      return objectType.getValue();
   }

   public void setObjectType(IsaacROSFoundationPoseObject objectType)
   {
      this.objectType.setValue(objectType);
   }

   public boolean getUseFoundationPose()
   {
      return useFoundationPose.getValue();
   }

   public void setUseFoundationPose(boolean useFoundationPose)
   {
      this.useFoundationPose.setValue(useFoundationPose);
   }

   public CRDTBidirectionalStringList getEnabledYoloModels()
   {
      return enabledYoloModels;
   }

   public CRDTBidirectionalIntegerList getIgnoredYoloClasses()
   {
      return ignoredYoloClassIndices;
   }

   public CRDTBidirectionalEnumList<IsaacROSFoundationPoseObject> getEnabledFoundationPoseModels()
   {
      return enabledFoundationPoseModels;
   }
}

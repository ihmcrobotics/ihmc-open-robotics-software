package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalBooleanArray;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerArray;
import us.ihmc.communication.crdt.CRDTBidirectionalStringArray;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;

public class SceneActionNodeDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalString yoloModelName;
   private final CRDTBidirectionalDouble yoloConfidenceThreshold;
   private final CRDTBidirectionalDouble yoloMaskThreshold;
   private final CRDTBidirectionalInteger segmentationMaskErosionRadius;
   private final CRDTBidirectionalDouble outlierThreshold;
   private final CRDTBidirectionalEnumField<IsaacROSFoundationPoseObject> objectType;
   private final CRDTBidirectionalBoolean useFoundationPose;
   private final CRDTBidirectionalStringArray enabledYoloModels;
   private final CRDTBidirectionalBooleanArray ignoredYoloClasses;
   private final CRDTBidirectionalIntegerArray enabledFoundationPoseModels;

   private String onDiskYoloModelName;
   private double onDiskYoloConfidenceThreshold;
   private double onDiskYoloMaskThreshold;
   private int onDiskSegmentationMaskErosionRadius;
   private double onDiskOutlierThreshold;
   private IsaacROSFoundationPoseObject onDiskObjectType;
   private boolean onDiskUseFoundationPose;
   private String[] onDiskEnabledYoloModels;
   private boolean[] onDiskIgnoredYoloClasses;
   private int[] onDiskEnabledFoundationPoseModels;

   public SceneActionNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      yoloModelName = new CRDTBidirectionalString(this, "");
      yoloConfidenceThreshold = new CRDTBidirectionalDouble(this, 0.5);
      yoloMaskThreshold = new CRDTBidirectionalDouble(this, 0.5);
      segmentationMaskErosionRadius = new CRDTBidirectionalInteger(this, 0);
      outlierThreshold = new CRDTBidirectionalDouble(this, 0.0);
      objectType = new CRDTBidirectionalEnumField<>(this, IsaacROSFoundationPoseObject.MUSTARD);
      useFoundationPose = new CRDTBidirectionalBoolean(this, false);
      enabledYoloModels = new CRDTBidirectionalStringArray(this, 10);
      ignoredYoloClasses = new CRDTBidirectionalBooleanArray(this, 256);
      enabledFoundationPoseModels = new CRDTBidirectionalIntegerArray(this, 10);
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
      for (int i = 0; i < enabledYoloModels.getLength(); i++)
      {
         String model = enabledYoloModels.getValueReadOnly(i);
         if (!model.isEmpty())
            enabledYoloModelsArray.add(model);
      }

      ArrayNode ignoredYoloClassesArray = jsonNode.putArray("ignoredYoloClasses");
      for (int i = 0; i < ignoredYoloClasses.getLength(); i++)
      {
         ignoredYoloClassesArray.add(ignoredYoloClasses.getValueReadOnly(i));
      }

      ArrayNode enabledFoundationPoseModelsArray = jsonNode.putArray("enabledFoundationPoseModels");
      for (int i = 0; i < enabledFoundationPoseModels.getLength(); i++)
      {
         int model = enabledFoundationPoseModels.getValueReadOnly(i);
         if (model != 0)
            enabledFoundationPoseModelsArray.add(model);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      if (jsonNode.has("yoloModelName"))
         yoloModelName.setValue(jsonNode.get("yoloModelName").asText());
      if (jsonNode.has("yoloConfidenceThreshold"))
         yoloConfidenceThreshold.setValue(jsonNode.get("yoloConfidenceThreshold").asDouble());
      if (jsonNode.has("yoloMaskThreshold"))
         yoloMaskThreshold.setValue(jsonNode.get("yoloMaskThreshold").asDouble());
      if (jsonNode.has("segmentationMaskErosionRadius"))
         segmentationMaskErosionRadius.setValue(jsonNode.get("segmentationMaskErosionRadius").asInt());
      if (jsonNode.has("outlierThreshold"))
         outlierThreshold.setValue(jsonNode.get("outlierThreshold").asDouble());
      if (jsonNode.has("objectType"))
         objectType.setValue(IsaacROSFoundationPoseObject.valueOf(jsonNode.get("objectType").asText()));
      if (jsonNode.has("useFoundationPose"))
         useFoundationPose.setValue(jsonNode.get("useFoundationPose").asBoolean());

      if (jsonNode.has("enabledYoloModels"))
      {
         ArrayNode enabledYoloModelsArray = (ArrayNode) jsonNode.get("enabledYoloModels");
         for (int i = 0; i < enabledYoloModelsArray.size() && i < enabledYoloModels.getLength(); i++)
         {
            enabledYoloModels.setValue(i, enabledYoloModelsArray.get(i).asText());
         }
      }

      if (jsonNode.has("ignoredYoloClasses"))
      {
         ArrayNode ignoredYoloClassesArray = (ArrayNode) jsonNode.get("ignoredYoloClasses");
         for (int i = 0; i < ignoredYoloClassesArray.size() && i < ignoredYoloClasses.getLength(); i++)
         {
            ignoredYoloClasses.setValue(i, ignoredYoloClassesArray.get(i).asBoolean());
         }
      }

      if (jsonNode.has("enabledFoundationPoseModels"))
      {
         ArrayNode enabledFoundationPoseModelsArray = (ArrayNode) jsonNode.get("enabledFoundationPoseModels");
         for (int i = 0; i < enabledFoundationPoseModelsArray.size() && i < enabledFoundationPoseModels.getLength(); i++)
         {
            enabledFoundationPoseModels.setValue(i, enabledFoundationPoseModelsArray.get(i).asInt());
         }
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

      onDiskEnabledYoloModels = new String[enabledYoloModels.getLength()];
      for (int i = 0; i < enabledYoloModels.getLength(); i++)
      {
         onDiskEnabledYoloModels[i] = enabledYoloModels.getValueReadOnly(i);
      }

      onDiskIgnoredYoloClasses = new boolean[ignoredYoloClasses.getLength()];
      for (int i = 0; i < ignoredYoloClasses.getLength(); i++)
      {
         onDiskIgnoredYoloClasses[i] = ignoredYoloClasses.getValueReadOnly(i);
      }

      onDiskEnabledFoundationPoseModels = new int[enabledFoundationPoseModels.getLength()];
      for (int i = 0; i < enabledFoundationPoseModels.getLength(); i++)
      {
         onDiskEnabledFoundationPoseModels[i] = enabledFoundationPoseModels.getValueReadOnly(i);
      }
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

         for (int i = 0; i < enabledYoloModels.getLength() && i < onDiskEnabledYoloModels.length; i++)
         {
            enabledYoloModels.setValue(i, onDiskEnabledYoloModels[i]);
         }

         for (int i = 0; i < ignoredYoloClasses.getLength() && i < onDiskIgnoredYoloClasses.length; i++)
         {
            ignoredYoloClasses.setValue(i, onDiskIgnoredYoloClasses[i]);
         }

         for (int i = 0; i < enabledFoundationPoseModels.getLength() && i < onDiskEnabledFoundationPoseModels.length; i++)
         {
            enabledFoundationPoseModels.setValue(i, onDiskEnabledFoundationPoseModels[i]);
         }
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

      for (int i = 0; i < enabledYoloModels.getLength(); i++)
      {
         unchanged &= enabledYoloModels.getValueReadOnly(i).equals(onDiskEnabledYoloModels[i]);
      }

      for (int i = 0; i < ignoredYoloClasses.getLength(); i++)
      {
         unchanged &= ignoredYoloClasses.getValueReadOnly(i) == onDiskIgnoredYoloClasses[i];
      }

      for (int i = 0; i < enabledFoundationPoseModels.getLength(); i++)
      {
         unchanged &= enabledFoundationPoseModels.getValueReadOnly(i) == onDiskEnabledFoundationPoseModels[i];
      }

      return !unchanged;
   }

   public void toMessage(SceneActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setYoloModelName(yoloModelName.getValue());
      message.setYoloConfidenceThreshold((float) yoloConfidenceThreshold.getValue());
      message.setYoloMaskThreshold((float) yoloMaskThreshold.getValue());
      message.setSegmentationMaskErosionRadius(segmentationMaskErosionRadius.getValue());
      message.setOutlierThreshold((float) outlierThreshold.getValue());
      message.setObjectType(objectType.toMessageOrdinal());
      message.setUseFoundationPose(useFoundationPose.getValue());

      enabledYoloModels.toMessage(message.getEnabledYoloModels());
      ignoredYoloClasses.toMessage(message.getIgnoredYoloClasses());
//      enabledFoundationPoseModels.toMessage(message.getEnabledFoundationPoseModels());
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
      ignoredYoloClasses.fromMessage(message.getIgnoredYoloClasses());
//      enabledFoundationPoseModels.fromMessage(message.getEnabledFoundationPoseModels());
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

   public void setYoloConfidenceThreshold(double yoloConfidenceThreshold)
   {
      this.yoloConfidenceThreshold.setValue(yoloConfidenceThreshold);
   }

   public double getYoloMaskThreshold()
   {
      return yoloMaskThreshold.getValue();
   }

   public void setYoloMaskThreshold(double yoloMaskThreshold)
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

   public double getOutlierThreshold()
   {
      return outlierThreshold.getValue();
   }

   public void setOutlierThreshold(double outlierThreshold)
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

   public CRDTBidirectionalStringArray getEnabledYoloModels()
   {
      return enabledYoloModels;
   }

   public CRDTBidirectionalBooleanArray getIgnoredYoloClasses()
   {
      return ignoredYoloClasses;
   }

   public CRDTBidirectionalIntegerArray getEnabledFoundationPoseModels()
   {
      return enabledFoundationPoseModels;
   }
}

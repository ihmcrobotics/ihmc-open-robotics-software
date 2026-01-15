package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerList;
import us.ihmc.communication.crdt.CRDTBidirectionalStringList;

import java.util.ArrayList;
import java.util.List;

public class SceneActionNodeDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalFloat timeout;
   private final CRDTBidirectionalInteger minimumHistorySize;
   private final BehaviorTreeSceneObjectDefinition sceneObjectDefinition;
   private final CRDTBidirectionalFloat yoloConfidenceThreshold;
   private final CRDTBidirectionalFloat yoloMaskThreshold;
   private final CRDTBidirectionalInteger segmentationMaskErosionRadius;
   private final CRDTBidirectionalFloat outlierThreshold;
   private final CRDTBidirectionalStringList enabledYoloModels;
   private final CRDTBidirectionalIntegerList ignoredYoloClassIndices;
   private final CRDTBidirectionalIntegerList enabledFoundationPoseModels;

   private float onDiskTimeout;
   private int onDiskMinimumHistorySize;
   private float onDiskYoloConfidenceThreshold;
   private float onDiskYoloMaskThreshold;
   private int onDiskSegmentationMaskErosionRadius;
   private float onDiskOutlierThreshold;
   private final List<String> onDiskEnabledYoloModels = new ArrayList<>();
   private final TIntArrayList onDiskIgnoredYoloClassIndices = new TIntArrayList();
   private final TIntArrayList onDiskEnabledFoundationPoseModels = new TIntArrayList();

   public SceneActionNodeDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      timeout = new CRDTBidirectionalFloat(this, 5.0f);
      minimumHistorySize = new CRDTBidirectionalInteger(this, 5);
      sceneObjectDefinition = new BehaviorTreeSceneObjectDefinition(this);
      yoloConfidenceThreshold = new CRDTBidirectionalFloat(this, 0.7f);
      yoloMaskThreshold = new CRDTBidirectionalFloat(this, 0.0f);
      segmentationMaskErosionRadius = new CRDTBidirectionalInteger(this, 1);
      outlierThreshold = new CRDTBidirectionalFloat(this, 2.0f);
      enabledYoloModels = new CRDTBidirectionalStringList(this);
      ignoredYoloClassIndices = new CRDTBidirectionalIntegerList(this);
      enabledFoundationPoseModels = new CRDTBidirectionalIntegerList(this);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("timeout", timeout.getValue());
      jsonNode.put("minimumHistorySize", minimumHistorySize.getValue());
      ObjectNode sceneObjectNode = jsonNode.putObject("sceneObjectDefinition");
      sceneObjectDefinition.saveToFile(sceneObjectNode);
      jsonNode.put("yoloConfidenceThreshold", yoloConfidenceThreshold.getValue());
      jsonNode.put("yoloMaskThreshold", yoloMaskThreshold.getValue());
      jsonNode.put("segmentationMaskErosionRadius", segmentationMaskErosionRadius.getValue());
      jsonNode.put("outlierThreshold", outlierThreshold.getValue());

      ArrayNode enabledYoloModelsArray = jsonNode.putArray("enabledYoloModels");
      for (int i = 0; i < enabledYoloModels.getSize(); i++)
         enabledYoloModelsArray.add(enabledYoloModels.getValueReadOnly(i));

      ArrayNode ignoredYoloClassesArray = jsonNode.putArray("ignoredYoloClassIndices");
      for (int i = 0; i < ignoredYoloClassIndices.getSize(); i++)
         ignoredYoloClassesArray.add(ignoredYoloClassIndices.getValueReadOnly(i));

      ArrayNode enabledFoundationPoseModelsArray = jsonNode.putArray("enabledFoundationPoseModels");
      for (int i = 0; i < enabledFoundationPoseModels.getSize(); i++)
         enabledFoundationPoseModelsArray.add(enabledFoundationPoseModels.getValueReadOnly(i));
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      timeout.setValue((float) jsonNode.get("timeout").asDouble());
      minimumHistorySize.setValue(jsonNode.get("minimumHistorySize").asInt());
      sceneObjectDefinition.loadFromFile(jsonNode.get("sceneObjectDefinition"));
      yoloConfidenceThreshold.setValue((float) jsonNode.get("yoloConfidenceThreshold").asDouble());
      yoloMaskThreshold.setValue((float) jsonNode.get("yoloMaskThreshold").asDouble());
      segmentationMaskErosionRadius.setValue(jsonNode.get("segmentationMaskErosionRadius").asInt());
      outlierThreshold.setValue((float) jsonNode.get("outlierThreshold").asDouble());

      ArrayNode enabledYoloModelsArray = (ArrayNode) jsonNode.get("enabledYoloModels");
      enabledYoloModels.clear();
      for (int i = 0; i < enabledYoloModelsArray.size(); i++)
         enabledYoloModels.setValue(i, enabledYoloModelsArray.get(i).asText());

      ArrayNode ignoredYoloClassesArray = (ArrayNode) jsonNode.get("ignoredYoloClassIndices");
      ignoredYoloClassIndices.clear();
      for (int i = 0; i < ignoredYoloClassesArray.size(); i++)
         ignoredYoloClassIndices.setValue(i, ignoredYoloClassesArray.get(i).asInt());

      ArrayNode enabledFoundationPoseModelsArray = (ArrayNode) jsonNode.get("enabledFoundationPoseModels");
      enabledFoundationPoseModels.clear();
      for (int i = 0; i < enabledFoundationPoseModelsArray.size(); i++)
         enabledFoundationPoseModels.setValue(i, enabledFoundationPoseModelsArray.get(i).asInt());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskTimeout = timeout.getValue();
      onDiskMinimumHistorySize = minimumHistorySize.getValue();
      sceneObjectDefinition.setOnDiskFields();
      onDiskYoloConfidenceThreshold = yoloConfidenceThreshold.getValue();
      onDiskYoloMaskThreshold = yoloMaskThreshold.getValue();
      onDiskSegmentationMaskErosionRadius = segmentationMaskErosionRadius.getValue();
      onDiskOutlierThreshold = outlierThreshold.getValue();

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
         timeout.setValue(onDiskTimeout);
         minimumHistorySize.setValue(onDiskMinimumHistorySize);
         sceneObjectDefinition.undoAllNontopologicalChanges();
         yoloConfidenceThreshold.setValue(onDiskYoloConfidenceThreshold);
         yoloMaskThreshold.setValue(onDiskYoloMaskThreshold);
         segmentationMaskErosionRadius.setValue(onDiskSegmentationMaskErosionRadius);
         outlierThreshold.setValue(onDiskOutlierThreshold);

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

      unchanged &= timeout.getValue() == onDiskTimeout;
      unchanged &= minimumHistorySize.getValue() == onDiskMinimumHistorySize;
      unchanged &= !sceneObjectDefinition.hasChanges();
      unchanged &= yoloConfidenceThreshold.getValue() == onDiskYoloConfidenceThreshold;
      unchanged &= yoloMaskThreshold.getValue() == onDiskYoloMaskThreshold;
      unchanged &= segmentationMaskErosionRadius.getValue() == onDiskSegmentationMaskErosionRadius;
      unchanged &= outlierThreshold.getValue() == onDiskOutlierThreshold;

      unchanged &= enabledYoloModels.getSize() == onDiskEnabledYoloModels.size();
      for (int i = 0; unchanged && i < enabledYoloModels.getSize(); i++)
         unchanged = enabledYoloModels.getValueReadOnly(i).equals(onDiskEnabledYoloModels.get(i));

      unchanged &= ignoredYoloClassIndices.getSize() == onDiskIgnoredYoloClassIndices.size();
      for (int i = 0; unchanged && i < ignoredYoloClassIndices.getSize(); i++)
         unchanged = ignoredYoloClassIndices.getValueReadOnly(i) == onDiskIgnoredYoloClassIndices.get(i);

      unchanged &= enabledFoundationPoseModels.getSize() == onDiskEnabledFoundationPoseModels.size();
      for (int i = 0; unchanged && i < enabledFoundationPoseModels.getSize(); i++)
         unchanged = enabledFoundationPoseModels.getValueReadOnly(i) == onDiskEnabledFoundationPoseModels.get(i);

      return !unchanged;
   }

   public void toMessage(SceneActionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setTimeout(timeout.toMessage());
      message.setMinimumHistorySize(minimumHistorySize.toMessage());
      sceneObjectDefinition.toMessage(message.getSceneObjectDefinition());
      message.setYoloConfidenceThreshold(yoloConfidenceThreshold.toMessage());
      message.setYoloMaskThreshold(yoloMaskThreshold.toMessage());
      message.setSegmentationMaskErosionRadius(segmentationMaskErosionRadius.toMessage());
      message.setOutlierThreshold(outlierThreshold.toMessage());
      enabledYoloModels.toMessage(message.getEnabledYoloModels());
      ignoredYoloClassIndices.toMessage(message.getIgnoredYoloClassIndices());
      enabledFoundationPoseModels.toMessage(message.getEnabledFoundationPoseModels());
   }

   public void fromMessage(SceneActionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      timeout.fromMessage(message.getTimeout());
      minimumHistorySize.fromMessage(message.getMinimumHistorySize());
      sceneObjectDefinition.fromMessage(message.getSceneObjectDefinition());
      yoloConfidenceThreshold.fromMessage(message.getYoloConfidenceThreshold());
      yoloMaskThreshold.fromMessage(message.getYoloMaskThreshold());
      segmentationMaskErosionRadius.fromMessage(message.getSegmentationMaskErosionRadius());
      outlierThreshold.fromMessage(message.getOutlierThreshold());
      enabledYoloModels.fromMessage(message.getEnabledYoloModels());
      ignoredYoloClassIndices.fromMessage(message.getIgnoredYoloClassIndices());
      enabledFoundationPoseModels.fromMessage(message.getEnabledFoundationPoseModels());
   }

   public BehaviorTreeSceneObjectDefinition getSceneObjectDefinition()
   {
      return sceneObjectDefinition;
   }

   public float getTimeout()
   {
      return timeout.getValue();
   }

   public void setTimeout(float timeout)
   {
      this.timeout.setValue(timeout);
   }

   public int getMinimumHistorySize()
   {
      return minimumHistorySize.getValue();
   }

   public void setMinimumHistorySize(int minimumHistorySize)
   {
      this.minimumHistorySize.setValue(minimumHistorySize);
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

   public CRDTBidirectionalStringList getEnabledYoloModels()
   {
      return enabledYoloModels;
   }

   public CRDTBidirectionalIntegerList getIgnoredYoloClasses()
   {
      return ignoredYoloClassIndices;
   }

   public CRDTBidirectionalIntegerList getEnabledFoundationPoseModels()
   {
      return enabledFoundationPoseModels;
   }
}

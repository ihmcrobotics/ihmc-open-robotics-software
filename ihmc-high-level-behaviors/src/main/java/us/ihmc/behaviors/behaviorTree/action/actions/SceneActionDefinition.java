package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerList;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.CRDTBidirectionalStringList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

import java.util.ArrayList;
import java.util.List;

public class SceneActionDefinition extends ActionNodeDefinition
{
   public enum SceneActionType
   {
      SETUP_OBJECT,
      FREEZE_OBJECT,
      DELETE_OBJECT,
      CLEAR_SCENE,
      FREEZE_SCENE,
      CONFIGURE_YOLO,
      CONFIGURE_FOUNDATION_POSE,
      CREATE_FRAME;

      public static final SceneActionType[] values = values();
   }

   private final CRDTBidirectionalEnumField<SceneActionType> sceneActionType;
   private final BehaviorTreeSceneObjectDefinition sceneObjectDefinition;
   private final CRDTBidirectionalFloat timeout;
   private final CRDTBidirectionalInteger minimumHistorySize;
   private final CRDTBidirectionalRigidBodyTransform nominalObjectPose;
   private final CRDTBidirectionalFloat yoloConfidenceThreshold;
   private final CRDTBidirectionalFloat yoloMaskThreshold;
   private final CRDTBidirectionalInteger segmentationMaskErosionRadius;
   private final CRDTBidirectionalFloat outlierThreshold;
   private final CRDTBidirectionalStringList enabledYoloModels;
   private final CRDTBidirectionalIntegerList ignoredYoloClassIndices;
   private final CRDTBidirectionalIntegerList enabledFoundationPoseModels;
   private final CRDTBidirectionalString frameName;
   private final CRDTBidirectionalString frameA;
   private final CRDTBidirectionalString frameB;
   private final CRDTBidirectionalFloat distance;

   private SceneActionType onDiskSceneActionType;
   private float onDiskTimeout;
   private int onDiskMinimumHistorySize;
   private final RigidBodyTransform onDiskNominalObjectPose = new RigidBodyTransform();
   private float onDiskYoloConfidenceThreshold;
   private float onDiskYoloMaskThreshold;
   private int onDiskSegmentationMaskErosionRadius;
   private float onDiskOutlierThreshold;
   private final List<String> onDiskEnabledYoloModels = new ArrayList<>();
   private final TIntArrayList onDiskIgnoredYoloClassIndices = new TIntArrayList();
   private int onDiskFoundationPoseObjectType;
   private final TIntArrayList onDiskEnabledFoundationPoseModels = new TIntArrayList();
   private String onDiskFrameName;
   private String onDiskFrameA;
   private String onDiskFrameB;
   private float onDiskDistance;

   public SceneActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      sceneActionType = new CRDTBidirectionalEnumField<>(this, SceneActionType.SETUP_OBJECT);
      sceneObjectDefinition = new BehaviorTreeSceneObjectDefinition(this);
      timeout = new CRDTBidirectionalFloat(this, 5.0f);
      minimumHistorySize = new CRDTBidirectionalInteger(this, 5);
      nominalObjectPose = new CRDTBidirectionalRigidBodyTransform(this);
      yoloConfidenceThreshold = new CRDTBidirectionalFloat(this, 0.7f);
      yoloMaskThreshold = new CRDTBidirectionalFloat(this, 0.0f);
      segmentationMaskErosionRadius = new CRDTBidirectionalInteger(this, 1);
      outlierThreshold = new CRDTBidirectionalFloat(this, 2.0f);
      enabledYoloModels = new CRDTBidirectionalStringList(this);
      ignoredYoloClassIndices = new CRDTBidirectionalIntegerList(this);
      enabledFoundationPoseModels = new CRDTBidirectionalIntegerList(this);
      frameName = new CRDTBidirectionalString(this, "");
      frameA = new CRDTBidirectionalString(this, "");
      frameB = new CRDTBidirectionalString(this, "");
      distance = new CRDTBidirectionalFloat(this, 0.0f);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("sceneActionType", sceneActionType.getValue().name());

      switch (sceneActionType.getValue())
      {
         case SETUP_OBJECT ->
         {
            ObjectNode sceneObjectNode = jsonNode.putObject("sceneObjectDefinition");
            sceneObjectDefinition.saveToFile(sceneObjectNode);
            jsonNode.put("timeout", timeout.getValue());
            jsonNode.put("minimumHistorySize", minimumHistorySize.getValue());
            JSONTools.toJSON(jsonNode.putObject("nominalObjectPose"), nominalObjectPose.getValueReadOnly());
         }
         case CONFIGURE_YOLO ->
         {
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
         }
         case CONFIGURE_FOUNDATION_POSE ->
         {
            ArrayNode enabledFoundationPoseModelsArray = jsonNode.putArray("enabledFoundationPoseModels");
            for (int i = 0; i < enabledFoundationPoseModels.getSize(); i++)
               enabledFoundationPoseModelsArray.add(enabledFoundationPoseModels.getValueReadOnly(i));
         }
         case CREATE_FRAME ->
         {
            jsonNode.put("frameName", frameName.getValue());
            jsonNode.put("frameA", frameA.getValue());
            jsonNode.put("frameB", frameB.getValue());
            jsonNode.put("distance", distance.getValue());
         }
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      sceneActionType.setValue(SceneActionType.valueOf(jsonNode.get("sceneActionType").textValue()));

      switch (sceneActionType.getValue())
      {
         case SETUP_OBJECT ->
         {
            sceneObjectDefinition.loadFromFile(jsonNode.get("sceneObjectDefinition"));
            timeout.setValue((float) jsonNode.get("timeout").asDouble());
            minimumHistorySize.setValue(jsonNode.get("minimumHistorySize").asInt());
            if (jsonNode.get("nominalObjectPose") instanceof ObjectNode nominalObjectPoseNode)
               JSONTools.toEuclid(nominalObjectPoseNode, nominalObjectPose.getValueAndModify());
         }
         case CONFIGURE_YOLO ->
         {
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
         }
         case CONFIGURE_FOUNDATION_POSE ->
         {
            ArrayNode enabledFoundationPoseModelsArray = (ArrayNode) jsonNode.get("enabledFoundationPoseModels");
            enabledFoundationPoseModels.clear();
            for (int i = 0; i < enabledFoundationPoseModelsArray.size(); i++)
               enabledFoundationPoseModels.setValue(i, enabledFoundationPoseModelsArray.get(i).asInt());
         }
         case CREATE_FRAME ->
         {
            frameName.setValue(jsonNode.has("frameName") ? jsonNode.get("frameName").asText() : "");
            frameA.setValue(jsonNode.has("frameA") ? jsonNode.get("frameA").asText() : "");
            frameB.setValue(jsonNode.has("frameB") ? jsonNode.get("frameB").asText() : "");
            distance.setValue(jsonNode.has("distance") ? (float) jsonNode.get("distance").asDouble() : 0.0f);
         }
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSceneActionType = sceneActionType.getValue();
      sceneObjectDefinition.setOnDiskFields();
      onDiskTimeout = timeout.getValue();
      onDiskMinimumHistorySize = minimumHistorySize.getValue();
      onDiskNominalObjectPose.set(nominalObjectPose.getValueReadOnly());
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
      onDiskFrameName = frameName.getValue();
      onDiskFrameA = frameA.getValue();
      onDiskFrameB = frameB.getValue();
      onDiskDistance = distance.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         sceneActionType.setValue(onDiskSceneActionType);
         sceneObjectDefinition.undoAllNontopologicalChanges();
         timeout.setValue(onDiskTimeout);
         minimumHistorySize.setValue(onDiskMinimumHistorySize);
         nominalObjectPose.getValueAndModify().set(onDiskNominalObjectPose);
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
         frameName.setValue(onDiskFrameName);
         frameA.setValue(onDiskFrameA);
         frameB.setValue(onDiskFrameB);
         distance.setValue(onDiskDistance);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= sceneActionType.getValue() == onDiskSceneActionType;
      unchanged &= !sceneObjectDefinition.hasChanges();
      unchanged &= timeout.getValue() == onDiskTimeout;
      unchanged &= minimumHistorySize.getValue() == onDiskMinimumHistorySize;
      unchanged &= nominalObjectPose.getValueReadOnly().equals(onDiskNominalObjectPose);
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
      unchanged &= frameName.getValue().equals(onDiskFrameName);
      unchanged &= frameA.getValue().equals(onDiskFrameA);
      unchanged &= frameB.getValue().equals(onDiskFrameB);
      unchanged &= distance.getValue() == onDiskDistance;

      return !unchanged;
   }

   public void toMessage(SceneActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSceneActionType(sceneActionType.toMessageOrdinal());
      sceneObjectDefinition.toMessage(message.getSceneObjectDefinition());
      message.setTimeout(timeout.toMessage());
      message.setMinimumHistorySize(minimumHistorySize.toMessage());
      nominalObjectPose.toMessage(message.getNominalObjectPose());
      message.setYoloConfidenceThreshold(yoloConfidenceThreshold.toMessage());
      message.setYoloMaskThreshold(yoloMaskThreshold.toMessage());
      message.setSegmentationMaskErosionRadius(segmentationMaskErosionRadius.toMessage());
      message.setOutlierThreshold(outlierThreshold.toMessage());
      enabledYoloModels.toMessage(message.getEnabledYoloModels());
      ignoredYoloClassIndices.toMessage(message.getIgnoredYoloClassIndices());
      enabledFoundationPoseModels.toMessage(message.getEnabledFoundationPoseModels());
      message.setFrameName(frameName.toMessage());
      message.setFrameA(frameA.toMessage());
      message.setFrameB(frameB.toMessage());
      message.setDistance(distance.toMessage());
   }

   public void fromMessage(SceneActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      sceneActionType.fromMessageOrdinal(message.getSceneActionType(), SceneActionType.values);
      sceneObjectDefinition.fromMessage(message.getSceneObjectDefinition());
      timeout.fromMessage(message.getTimeout());
      minimumHistorySize.fromMessage(message.getMinimumHistorySize());
      nominalObjectPose.fromMessage(message.getNominalObjectPose());
      yoloConfidenceThreshold.fromMessage(message.getYoloConfidenceThreshold());
      yoloMaskThreshold.fromMessage(message.getYoloMaskThreshold());
      segmentationMaskErosionRadius.fromMessage(message.getSegmentationMaskErosionRadius());
      outlierThreshold.fromMessage(message.getOutlierThreshold());
      enabledYoloModels.fromMessage(message.getEnabledYoloModels());
      ignoredYoloClassIndices.fromMessage(message.getIgnoredYoloClassIndices());
      enabledFoundationPoseModels.fromMessage(message.getEnabledFoundationPoseModels());
      frameName.fromMessage(message.getFrameNameAsString());
      frameA.fromMessage(message.getFrameAAsString());
      frameB.fromMessage(message.getFrameBAsString());
      distance.fromMessage(message.getDistance());
   }

   public BehaviorTreeSceneObjectDefinition getSceneObjectDefinition()
   {
      return sceneObjectDefinition;
   }

   public CRDTBidirectionalEnumField<SceneActionType> getSceneActionType()
   {
      return sceneActionType;
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

   public CRDTBidirectionalRigidBodyTransform getNominalObjectPose()
   {
      return nominalObjectPose;
   }

   public float getYoloConfidenceThreshold()
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

   public String getFrameName()
   {
      return frameName.getValue();
   }

   public void setFrameName(String frameName)
   {
      this.frameName.setValue(frameName);
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
}

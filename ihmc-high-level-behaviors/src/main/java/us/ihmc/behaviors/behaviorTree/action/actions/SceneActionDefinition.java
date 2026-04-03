package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.SceneActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.list.array.TIntArrayList;
import org.yaml.snakeyaml.Yaml;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectDefinition;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.crdt.CRDTBidirectionalEnumField;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerList;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ModelParameters;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.tools.io.JSONTools;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;

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
      CONFIGURE_FOUNDATION_POSE;

      public static final SceneActionType[] values = values();
   }

   private final YOLOv8ModelParametersDefinition[] syncableYOLOModelParameters;

   private final CRDTBidirectionalEnumField<SceneActionType> sceneActionType;
   private final BehaviorTreeSceneObjectDefinition sceneObjectDefinition;
   private final CRDTBidirectionalFloat timeout;
   private final CRDTBidirectionalInteger minimumHistorySize;
   private final CRDTBidirectionalRigidBodyTransform nominalObjectPose;
   private final CRDTBidirectionalIntegerList enabledYoloModels;
   private final CRDTBidirectionalIntegerList enabledFoundationPoseModels;

   private SceneActionType onDiskSceneActionType;
   private float onDiskTimeout;
   private int onDiskMinimumHistorySize;
   private final RigidBodyTransform onDiskNominalObjectPose = new RigidBodyTransform();
   private final TIntArrayList onDiskEnabledYoloModels = new TIntArrayList();
   private final TIntArrayList onDiskEnabledFoundationPoseModels = new TIntArrayList();

   public SceneActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      List<URL> yoloModelDirectories = YOLOv8Tools.getYOLOModelDirectories();
      YOLOv8ModelInfo[] availableYOLOModels = new YOLOv8ModelInfo[yoloModelDirectories.size()];
      for (int i = 0; i < yoloModelDirectories.size(); i++)
      {
         String[] path = yoloModelDirectories.get(i).getPath().split(Pattern.quote(File.separator));
         availableYOLOModels[i] = new YOLOv8ModelInfo();
         availableYOLOModels[i].setModelName(path[path.length - 1]);
      }
      for (int i = 0; i < availableYOLOModels.length; i++)
      {
         try (InputStream classNamesFile = YOLOv8Tools.getClassNamesFile(yoloModelDirectories.get(i)).openStream())
         {
            Yaml yaml = new Yaml();
            Map<String, List<Object>> classNamesData = yaml.load(classNamesFile);
            List<Object> names = classNamesData.get("names");
            for (Object name : names)
               availableYOLOModels[i].getDetectableObjectClasses().add(name.toString());
         }
         catch (IOException e)
         {
            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(e);
         }
      }
      syncableYOLOModelParameters = new YOLOv8ModelParametersDefinition[availableYOLOModels.length];
      for (int i = 0; i < availableYOLOModels.length; i++)
         syncableYOLOModelParameters[i] = new YOLOv8ModelParametersDefinition(this, availableYOLOModels[i]);

      sceneActionType = new CRDTBidirectionalEnumField<>(this, SceneActionType.SETUP_OBJECT);
      sceneObjectDefinition = new BehaviorTreeSceneObjectDefinition(this);
      timeout = new CRDTBidirectionalFloat(this, 5.0f);
      minimumHistorySize = new CRDTBidirectionalInteger(this, 5);
      nominalObjectPose = new CRDTBidirectionalRigidBodyTransform(this);
      enabledYoloModels = new CRDTBidirectionalIntegerList(this);
      enabledFoundationPoseModels = new CRDTBidirectionalIntegerList(this);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("sceneActionType", sceneActionType.getValue().name());

      switch (sceneActionType.getValue())
      {
         case SETUP_OBJECT, FREEZE_OBJECT, DELETE_OBJECT ->
         {
            ObjectNode sceneObjectNode = jsonNode.putObject("sceneObjectDefinition");
            sceneObjectDefinition.saveToFile(sceneObjectNode);
            jsonNode.put("timeout", timeout.getValue());
            jsonNode.put("minimumHistorySize", minimumHistorySize.getValue());
            JSONTools.toJSON(jsonNode.putObject("nominalObjectPose"), nominalObjectPose.getValueReadOnly());
         }
         case CONFIGURE_YOLO ->
         {
            ArrayNode enabledYoloModelsArray = jsonNode.putArray("enabledYoloModels");
            for (int i = 0; i < enabledYoloModels.getSize(); i++)
            {
               ObjectNode enabledYoloModelsNode = enabledYoloModelsArray.addObject();
               enabledYoloModelsNode.put("name", syncableYOLOModelParameters[enabledYoloModels.getValueReadOnly(i)].getModelName());
               syncableYOLOModelParameters[enabledYoloModels.getValueReadOnly(i)].saveToFile(enabledYoloModelsNode.putObject("parameters"));
            }
         }
         case CONFIGURE_FOUNDATION_POSE ->
         {
            ArrayNode enabledFoundationPoseModelsArray = jsonNode.putArray("enabledFoundationPoseModels");
            for (int i = 0; i < enabledFoundationPoseModels.getSize(); i++)
               enabledFoundationPoseModelsArray.add(enabledFoundationPoseModels.getValueReadOnly(i));
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
         case SETUP_OBJECT, FREEZE_OBJECT, DELETE_OBJECT ->
         {
            sceneObjectDefinition.loadFromFile(jsonNode.get("sceneObjectDefinition"));
            timeout.setValue((float) jsonNode.get("timeout").asDouble());
            minimumHistorySize.setValue(jsonNode.get("minimumHistorySize").asInt());
            if (jsonNode.get("nominalObjectPose") instanceof ObjectNode nominalObjectPoseNode)
               JSONTools.toEuclid(nominalObjectPoseNode, nominalObjectPose.getValueAndModify());
         }
         case CONFIGURE_YOLO ->
         {
            enabledYoloModels.clear();
            if (jsonNode.get("enabledYoloModels") instanceof ArrayNode enabledYoloModelsArray)
            {
               for (int i = 0; i < enabledYoloModelsArray.size(); i++)
               {
                  if (!(enabledYoloModelsArray.get(i) instanceof ObjectNode enabledYoloModelsNode))
                     continue;

                  String modelName = enabledYoloModelsNode.path("name").asText("");
                  for (int j = 0; j < syncableYOLOModelParameters.length; j++)
                  {
                     if (syncableYOLOModelParameters[j].getModelName().equals(modelName))
                     {
                        enabledYoloModels.add(j);
                        if (enabledYoloModelsNode.get("parameters") instanceof ObjectNode parametersNode)
                           syncableYOLOModelParameters[j].loadFromFile(parametersNode);
                        break;
                     }
                  }
               }
            }
         }
         case CONFIGURE_FOUNDATION_POSE ->
         {
            ArrayNode enabledFoundationPoseModelsArray = (ArrayNode) jsonNode.get("enabledFoundationPoseModels");
            enabledFoundationPoseModels.clear();
            for (int i = 0; i < enabledFoundationPoseModelsArray.size(); i++)
               enabledFoundationPoseModels.setValue(i, enabledFoundationPoseModelsArray.get(i).asInt());
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

      onDiskEnabledYoloModels.clear();
      onDiskEnabledYoloModels.addAll(enabledYoloModels.getValue());
      for (int i = 0; i < syncableYOLOModelParameters.length; i++)
         syncableYOLOModelParameters[i].setOnDiskFields();

      onDiskEnabledFoundationPoseModels.clear();
      onDiskEnabledFoundationPoseModels.addAll(enabledFoundationPoseModels.getValue());
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

         enabledYoloModels.clear();
         enabledYoloModels.getValue().addAll(onDiskEnabledYoloModels);
         for (int i = 0; i < syncableYOLOModelParameters.length; i++)
            syncableYOLOModelParameters[i].undoAllNontopologicalChanges();

         enabledFoundationPoseModels.clear();
         enabledFoundationPoseModels.getValue().addAll(onDiskEnabledFoundationPoseModels);
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

      unchanged &= enabledYoloModels.getSize() == onDiskEnabledYoloModels.size();
      for (int i = 0; unchanged && i < enabledYoloModels.getSize(); i++)
         unchanged = enabledYoloModels.getValueReadOnly(i) == onDiskEnabledYoloModels.get(i);
      for (int i = 0; unchanged && i < syncableYOLOModelParameters.length; i++)
         unchanged = !syncableYOLOModelParameters[i].hasChanges();

      unchanged &= enabledFoundationPoseModels.getSize() == onDiskEnabledFoundationPoseModels.size();
      for (int i = 0; unchanged && i < enabledFoundationPoseModels.getSize(); i++)
         unchanged = enabledFoundationPoseModels.getValueReadOnly(i) == onDiskEnabledFoundationPoseModels.get(i);

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

      enabledYoloModels.toMessage(message.getEnabledYoloModels());

      message.getYoloModelParameters().clear();
      for (int i = 0; i < enabledYoloModels.getSize(); i++)
         syncableYOLOModelParameters[enabledYoloModels.getValueReadOnly(i)].toMessage(message.getYoloModelParameters().add());

      enabledFoundationPoseModels.toMessage(message.getEnabledFoundationPoseModels());
   }

   public void fromMessage(SceneActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      sceneActionType.fromMessageOrdinal(message.getSceneActionType(), SceneActionType.values);
      sceneObjectDefinition.fromMessage(message.getSceneObjectDefinition());
      timeout.fromMessage(message.getTimeout());
      minimumHistorySize.fromMessage(message.getMinimumHistorySize());
      nominalObjectPose.fromMessage(message.getNominalObjectPose());

      enabledYoloModels.fromMessage(message.getEnabledYoloModels());
      for (int i = 0; i < enabledYoloModels.getSize() && i < message.getYoloModelParameters().size(); i++)
         syncableYOLOModelParameters[enabledYoloModels.getValueReadOnly(i)].fromMessage(message.getYoloModelParameters().get(i));

      enabledFoundationPoseModels.fromMessage(message.getEnabledFoundationPoseModels());
   }

   public SyncedYOLOv8ModelParameters[] getSyncableYOLOModelParameters()
   {
      return syncableYOLOModelParameters;
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

   public CRDTBidirectionalIntegerList getEnabledYoloModels()
   {
      return enabledYoloModels;
   }

   public CRDTBidirectionalIntegerList getEnabledFoundationPoseModels()
   {
      return enabledFoundationPoseModels;
   }
}

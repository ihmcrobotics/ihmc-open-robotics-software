package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import perception_msgs.msg.dds.YOLOv8ModelParameters;
import us.ihmc.communication.crdt.CRDTBidirectionalSet;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSet;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class SyncedYOLOv8ExecutorParameters extends LatestTimestampModifiable
{
   private final CRDTStatusSet<YOLOv8ModelInfo> availableModels;
   private final CRDTBidirectionalSet<String> modelsToRun;
   private final Map<String, SyncedYOLOv8ModelParameters> modelParameters;

   public SyncedYOLOv8ExecutorParameters(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      availableModels = new CRDTStatusSet<>(ROS2ActorDesignation.ROBOT, crdtInfo);
      modelsToRun = new CRDTBidirectionalSet<>(this);
      modelParameters = new HashMap<>();
   }

   public void setAvailableModels(Collection<YOLOv8Model> models)
   {
      availableModels.clear();
      availableModels.addAll(models.stream().map(YOLOv8Tools::toMessage).collect(Collectors.toSet()));
      updateModelSettings();
   }

   public CRDTStatusSet<YOLOv8ModelInfo> getAvailableModels()
   {
      return availableModels;
   }

   public CRDTBidirectionalSet<String> getModelsToRun()
   {
      return modelsToRun;
   }

   public Map<String, SyncedYOLOv8ModelParameters> getModelParameters()
   {
      return modelParameters;
   }

   public void toMessage(YOLOv8ExecutorParameters messageToPack)
   {
      toMessage(messageToPack.getLatestTimestampModifiable());

      messageToPack.getAvailableYoloModels().clear();
      availableModels.getReadOnly().forEach(model -> messageToPack.getAvailableYoloModels().add().set(model));

      messageToPack.getModelsToRun().clear();
      modelsToRun.getValue().forEach(model -> messageToPack.getModelsToRun().add(model));

      messageToPack.getModelSettings().clear();
      for (SyncedYOLOv8ModelParameters modelSetting : modelParameters.values())
      {
         modelSetting.toMessage(messageToPack.getModelSettings().add());
      }
   }

   public void fromMessage(YOLOv8ExecutorParameters message)
   {
      fromMessage(message.getLatestTimestampModifiable());

      availableModels.fromMessage(models ->
      {
         models.clear();
         models.addAll(message.getAvailableYoloModels());
      });

      modelsToRun.fromMessage(models ->
      {
         models.clear();
         models.addAll(message.getModelsToRun().stream().map(StringBuilder::toString).toList());
      });

      if (availableModels.getSize() != modelParameters.size())
         updateModelSettings();

      for (YOLOv8ModelParameters modelSettingsMessage : message.getModelSettings())
      {
         modelParameters.get(modelSettingsMessage.getModelNameAsString()).fromMessage(modelSettingsMessage);
      }
   }

   private void updateModelSettings()
   {
      modelParameters.clear();
      availableModels.getReadOnly().forEach(model -> modelParameters.put(model.getModelNameAsString(), new SyncedYOLOv8ModelParameters(getCRDTInfo(), model)));
      modelsToRun.retainAll(availableModels.getReadOnly().stream().map(YOLOv8ModelInfo::getModelNameAsString).collect(Collectors.toSet()));
   }
}

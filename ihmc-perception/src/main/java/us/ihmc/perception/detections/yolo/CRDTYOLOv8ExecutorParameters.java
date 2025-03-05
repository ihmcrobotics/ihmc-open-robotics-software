package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ExecutorSettings;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import perception_msgs.msg.dds.YOLOv8ModelSettings;
import us.ihmc.communication.crdt.CRDTBidirectionalSet;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSet;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class CRDTYOLOv8ExecutorParameters extends LatestTimestampModifiable
{
   private final CRDTStatusSet<YOLOv8ModelInfo> availableModels;
   private final CRDTBidirectionalSet<String> modelsToRun;
   private final Map<String, CRDTYOLOv8ModelParameters> modelSettings;

   public CRDTYOLOv8ExecutorParameters(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      availableModels = new CRDTStatusSet<>(ROS2ActorDesignation.ROBOT, crdtInfo);
      modelsToRun = new CRDTBidirectionalSet<>(this);
      modelSettings = new HashMap<>();
   }

   public void setAvailableModels(Collection<YOLOv8Model> models)
   {
      availableModels.clear();
      availableModels.addAll(models.stream().map(YOLOv8Tools::toMessage).collect(Collectors.toSet()));
   }

   public void update()
   {
      checkModified();

      if (availableModels.pollHasStatus())
      {
         modelSettings.clear();
         availableModels.getCopy().forEach(model -> modelSettings.put(model.getModelNameAsString(), new CRDTYOLOv8ModelParameters(this, model)));
         modelsToRun.retainAll(availableModels.getCopy().stream().map(YOLOv8ModelInfo::getModelNameAsString).collect(Collectors.toSet()));
      }
   }

   public CRDTStatusSet<YOLOv8ModelInfo> getAvailableModels()
   {
      return availableModels;
   }

   public CRDTBidirectionalSet<String> getModelsToRun()
   {
      return modelsToRun;
   }

   public Map<String, CRDTYOLOv8ModelParameters> getModelSettings()
   {
      return modelSettings;
   }

   public void toMessage(YOLOv8ExecutorSettings messageToPack)
   {
      toMessage(messageToPack.getLatestTimestampModifiable());

      messageToPack.getAvailableYoloModels().clear();
      availableModels.getCopy().forEach(model -> messageToPack.getAvailableYoloModels().add().set(model));

      messageToPack.getModelsToRun().clear();
      modelsToRun.getValue().forEach(model -> messageToPack.getModelsToRun().add(model));

      messageToPack.getModelSettings().clear();
      for (CRDTYOLOv8ModelParameters modelSetting : modelSettings.values())
      {
         modelSetting.toMessage(messageToPack.getModelSettings().add());
      }
   }

   public void fromMessage(YOLOv8ExecutorSettings message)
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

      for (YOLOv8ModelSettings modelSettingsMessage : message.getModelSettings())
      {
         modelSettings.get(modelSettingsMessage.getModelNameAsString()).fromMessage(modelSettingsMessage);
      }
   }
}

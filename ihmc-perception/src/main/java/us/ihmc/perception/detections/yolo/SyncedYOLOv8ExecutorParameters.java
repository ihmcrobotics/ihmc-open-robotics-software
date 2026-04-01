package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import perception_msgs.msg.dds.YOLOv8ModelParameters;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTBidirectionalSet;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSet;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class SyncedYOLOv8ExecutorParameters extends LatestTimestampModifiable
{
   private final CRDTStatusSet<YOLOv8ModelInfo> availableModels;
   private final CRDTBidirectionalSet<String> modelsToRun;
   private final Map<String, SyncedYOLOv8ModelParameters> modelParameters;

   private final YOLOv8ExecutorParameters message;

   private final ROS2Publisher<YOLOv8ExecutorParameters> publisher;
   private final ROS2Subscription<YOLOv8ExecutorParameters> subscription;
   private final TypedNotification<YOLOv8ExecutorParameters> newMessageNotification;

   private final Throttler publishThrottler;

   public SyncedYOLOv8ExecutorParameters(ROS2Node ros2Node, CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      availableModels = new CRDTStatusSet<>(ROS2ActorDesignation.ROBOT, crdtInfo);
      modelsToRun = new CRDTBidirectionalSet<>(this);
      modelParameters = new HashMap<>();

      message = new YOLOv8ExecutorParameters();
      newMessageNotification = new TypedNotification<>();
      subscription = ros2Node.createSubscription2(PerceptionAPI.YOLO_PARAMETERS, newMessageNotification::set);
      publisher = ros2Node.createPublisher(PerceptionAPI.YOLO_PARAMETERS);

      publishThrottler = new Throttler().setFrequency(5.0);
   }

   public void checkModifiedAndUpdate()
   {
      checkModified();

      if (newMessageNotification.poll())
         fromMessage(newMessageNotification.read());

      if (publishThrottler.run() || pollNeedSendFullData())
      {
         toMessage(message);
         publisher.publish(message);
      }
   }

   public void setAvailableModels(Collection<YOLOv8Model> models)
   {
      synchronized (availableModels)
      {
         availableModels.clear();
         if (availableModels.addAll(models.stream().map(YOLOv8Tools::toMessage).collect(Collectors.toSet())))
            modify();
      }
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

   public void close()
   {
      publisher.remove();
      subscription.remove();
   }

   private void toMessage(YOLOv8ExecutorParameters messageToPack)
   {
      toMessage(messageToPack.getLatestTimestampModifiable());

      messageToPack.getAvailableYoloModels().clear();
      synchronized (availableModels)
      {
         availableModels.getReadOnly().forEach(model -> messageToPack.getAvailableYoloModels().add().set(model));
      }

      messageToPack.getModelsToRun().clear();
      synchronized (modelsToRun)
      {
         modelsToRun.getValue().forEach(model -> messageToPack.getModelsToRun().add(model));
      }

      messageToPack.getModelSettings().clear();
      synchronized (modelParameters)
      {
         for (SyncedYOLOv8ModelParameters modelSetting : modelParameters.values())
            modelSetting.toMessage(messageToPack.getModelSettings().add());
      }
   }

   private void fromMessage(YOLOv8ExecutorParameters message)
   {
      fromMessage(message.getLatestTimestampModifiable());

      // If the message asks for full data, we don't want to take their data
      if (message.getLatestTimestampModifiable().getFullDataNeeded())
         return;

      synchronized (availableModels)
      {
         availableModels.fromMessage(models ->
         {
            models.clear();
            models.addAll(message.getAvailableYoloModels());
         });
      }

      synchronized (modelsToRun)
      {
         modelsToRun.fromMessage(models ->
         {
            models.clear();
            models.addAll(message.getModelsToRun().stream().map(StringBuilder::toString).toList());
         });
      }


      if (availableModels.getSize() != modelParameters.size())
         updateModelSettings();

      synchronized (modelParameters)
      {
         for (YOLOv8ModelParameters modelSettingsMessage : message.getModelSettings())
         {
            modelParameters.get(modelSettingsMessage.getModelNameAsString()).fromMessage(modelSettingsMessage);
         }
      }
      confirmReceivedFullData();
   }

   private void updateModelSettings()
   {
      modelParameters.clear();
      availableModels.getReadOnly().forEach(model -> modelParameters.put(model.getModelNameAsString(), new SyncedYOLOv8ModelParameters(this, model)));
      modelsToRun.retainAll(availableModels.getReadOnly().stream().map(YOLOv8ModelInfo::getModelNameAsString).collect(Collectors.toSet()));
   }
}

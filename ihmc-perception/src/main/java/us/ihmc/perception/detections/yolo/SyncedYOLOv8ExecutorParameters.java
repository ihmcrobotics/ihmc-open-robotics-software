package us.ihmc.perception.detections.yolo;

import perception_msgs.YOLOv8ExecutorParameters;
import perception_msgs.YOLOv8ModelInfo;
import perception_msgs.YOLOv8ModelParameters;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTBidirectionalSet;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSet;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;

public class SyncedYOLOv8ExecutorParameters extends LatestTimestampModifiable
{
   private final CRDTStatusSet<YOLOv8ModelInfo> availableModels;
   private final CRDTBidirectionalSet<String> modelsToRun;
   private final Map<String, SyncedYOLOv8ModelParameters> modelParameters;

   private final YOLOv8ExecutorParameters message;

   private final ROS2Node ros2Node;
   private final ROS2Publisher<YOLOv8ExecutorParameters> publisher;
   private final ROS2Subscription<YOLOv8ExecutorParameters> subscription;
   private final TypedNotification<YOLOv8ExecutorParameters> newMessageNotification;

   private final Throttler publishThrottler;

   public SyncedYOLOv8ExecutorParameters(ROS2Node ros2Node, CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      this.ros2Node = ros2Node;
      availableModels = new CRDTStatusSet<>(ROS2ActorDesignation.ROBOT, crdtInfo, LinkedHashSet::new);
      modelsToRun = new CRDTBidirectionalSet<>(this);
      modelParameters = new LinkedHashMap<>();

      message = new YOLOv8ExecutorParameters();
      newMessageNotification = new TypedNotification<>();
      subscription = ros2Node.createSubscription(PerceptionAPI.YOLO_PARAMETERS, reader ->
      {
         YOLOv8ExecutorParameters message = reader.read();
         if (message != null)
            newMessageNotification.set(message);
      });
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
         boolean modify = false;
         for (YOLOv8Model model : models)
            if (availableModels.add(YOLOv8Tools.toMessage(model)))
               modify = true;
         if (modify)
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
      synchronized (modelParameters)
      {
         return new LinkedHashMap<>(modelParameters);
      }
   }

   public SyncedYOLOv8ModelParameters getModelParameters(String modelName)
   {
      synchronized (modelParameters)
      {
         return modelParameters.get(modelName);
      }
   }

   public void close()
   {
      ros2Node.destroyPublisher(publisher);
      ros2Node.destroySubscription(subscription);
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
            for (YOLOv8ModelInfo m : message.getAvailableYoloModels())
               models.add(m);
         });
      }

      synchronized (modelsToRun)
      {
         modelsToRun.fromMessage(models ->
         {
            models.clear();
            for (int i = 0; i < message.getModelsToRun().size(); i++)
               models.add(message.getModelsToRun().getAsString(i));
         });
      }

      updateModelSettings();

      synchronized (modelParameters)
      {
         for (YOLOv8ModelParameters modelSettingsMessage : message.getModelSettings())
         {
            SyncedYOLOv8ModelParameters settings = modelParameters.get(modelSettingsMessage.getModelNameAsString());
            if (settings != null)
               settings.fromMessage(modelSettingsMessage);
            else
               LogTools.warn("Ignoring YOLO settings for unknown model: {}", modelSettingsMessage.getModelNameAsString());
         }
      }
      confirmReceivedFullData();
   }

   private void updateModelSettings()
   {
      List<YOLOv8ModelInfo> availableModelsInOrder = new ArrayList<>();
      synchronized (availableModels)
      {
         availableModelsInOrder.addAll(availableModels.getReadOnly());
      }

      LinkedHashMap<String, SyncedYOLOv8ModelParameters> orderedModelParameters = new LinkedHashMap<>();
      synchronized (modelParameters)
      {
         for (YOLOv8ModelInfo model : availableModelsInOrder)
         {
            String modelName = model.getModelNameAsString();
            SyncedYOLOv8ModelParameters existingParameters = modelParameters.get(modelName);
            orderedModelParameters.put(modelName, existingParameters != null ? existingParameters : new SyncedYOLOv8ModelParameters(this, model));
         }
         modelParameters.clear();
         modelParameters.putAll(orderedModelParameters);
      }

      modelsToRun.retainAll(orderedModelParameters.keySet());
   }
}

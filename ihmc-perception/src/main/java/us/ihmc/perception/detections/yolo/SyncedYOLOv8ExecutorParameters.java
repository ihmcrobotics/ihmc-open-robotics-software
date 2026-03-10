package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSet;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.Collection;
import java.util.stream.Collectors;

public class SyncedYOLOv8ExecutorParameters extends LatestTimestampModifiable
{
   private final CRDTStatusSet<YOLOv8ModelInfo> availableModels;
   private final CRDTBidirectionalString modelToRun;
   private final SyncedYOLOv8ModelParameters modelParameters;

   private final YOLOv8ExecutorParameters message;

   private final ROS2Publisher<YOLOv8ExecutorParameters> publisher;
   private final ROS2Subscription<YOLOv8ExecutorParameters> subscription;
   private final TypedNotification<YOLOv8ExecutorParameters> newMessageNotification;

   private boolean firstUpdate = true;

   public SyncedYOLOv8ExecutorParameters(ROS2Node ros2Node, CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      availableModels = new CRDTStatusSet<>(ROS2ActorDesignation.ROBOT, crdtInfo);
      modelToRun = new CRDTBidirectionalString(this, null);
      modelParameters = new SyncedYOLOv8ModelParameters(crdtInfo);

      message = new YOLOv8ExecutorParameters();
      newMessageNotification = new TypedNotification<>();
      subscription = ros2Node.createSubscription2(PerceptionAPI.YOLO_PARAMETERS, newMessageNotification::set);
      publisher = ros2Node.createPublisher(PerceptionAPI.YOLO_PARAMETERS);

      requestSendFullData();
   }

   public void checkModifiedAndUpdate()
   {
      checkModified();

      if (newMessageNotification.poll())
         fromMessage(newMessageNotification.read());

      if (pollNeedSendFullData() || getModelParameters().pollNeedSendFullData() || firstUpdate)
      {
         toMessage(message);
         publisher.publish(message);
      }

      firstUpdate = false;
   }

   public synchronized void setAvailableModels(Collection<YOLOv8Model> models)
   {
      availableModels.clear();
      if (availableModels.addAll(models.stream().map(YOLOv8Tools::toMessage).collect(Collectors.toSet())))
         modify();
   }

   public CRDTStatusSet<YOLOv8ModelInfo> getAvailableModels()
   {
      return availableModels;
   }

   public CRDTBidirectionalString getModelToRun()
   {
      return modelToRun;
   }

   public SyncedYOLOv8ModelParameters getModelParameters()
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
      availableModels.getReadOnly().forEach(model -> messageToPack.getAvailableYoloModels().add().set(model));

      messageToPack.setModelToRun(modelToRun.getValue());

      modelParameters.toMessage(messageToPack.getModelSettings());
   }

   private void fromMessage(YOLOv8ExecutorParameters message)
   {
      fromMessage(message.getLatestTimestampModifiable());

      availableModels.fromMessage(models ->
      {
         models.clear();
         models.addAll(message.getAvailableYoloModels());
      });

      modelToRun.fromMessage(message.getModelToRunAsString());

      modelParameters.fromMessage(message.getModelSettings());

      confirmReceivedFullData();
   }
}

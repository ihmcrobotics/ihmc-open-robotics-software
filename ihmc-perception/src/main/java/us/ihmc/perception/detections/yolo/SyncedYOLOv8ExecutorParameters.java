package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusSet;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.Collection;
import java.util.stream.Collectors;

public class SyncedYOLOv8ExecutorParameters extends LatestTimestampModifiable
{
   private final CRDTStatusSet<YOLOv8ModelInfo> availableModels;
   private final CRDTBidirectionalString modelToRun;
   private final SyncedYOLOv8ModelParameters modelParameters;

   public SyncedYOLOv8ExecutorParameters(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      availableModels = new CRDTStatusSet<>(ROS2ActorDesignation.ROBOT, crdtInfo);
      modelToRun = new CRDTBidirectionalString(this, null);
      modelParameters = new SyncedYOLOv8ModelParameters(crdtInfo);
   }

   public void setAvailableModels(Collection<YOLOv8Model> models)
   {
      availableModels.clear();
      availableModels.addAll(models.stream().map(YOLOv8Tools::toMessage).collect(Collectors.toSet()));
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

   public void toMessage(YOLOv8ExecutorParameters messageToPack)
   {
      toMessage(messageToPack.getLatestTimestampModifiable());

      messageToPack.getAvailableYoloModels().clear();
      availableModels.getReadOnly().forEach(model -> messageToPack.getAvailableYoloModels().add().set(model));

      messageToPack.setModelToRun(modelToRun.getValue());

      modelParameters.toMessage(messageToPack.getModelSettings());
   }

   public void fromMessage(YOLOv8ExecutorParameters message)
   {
      fromMessage(message.getLatestTimestampModifiable());

      availableModels.fromMessage(models ->
      {
         models.clear();
         models.addAll(message.getAvailableYoloModels());
      });

      modelToRun.fromMessage(message.getModelToRunAsString());

      modelParameters.fromMessage(message.getModelSettings());
   }
}

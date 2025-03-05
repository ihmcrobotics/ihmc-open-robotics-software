package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ExecutorSettings;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.communication.crdt.CRDTBidirectionalStringArray;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

public class CRDTYOLOv8ExecutorParameters
{
   private final LatestTimestampModifiable latestTimestampModifiable;

   private final CRDTBidirectionalStringArray modelsToRun;
   private final CRDTYOLOv8ModelParameters[] modelSettings;

   public CRDTYOLOv8ExecutorParameters(CRDTInfo crdtInfo, YOLOv8ModelInfo[] models)
   {
      latestTimestampModifiable = new LatestTimestampModifiable(crdtInfo);
      latestTimestampModifiable.setModifierName(getClass().getSimpleName());

      modelsToRun = new CRDTBidirectionalStringArray(latestTimestampModifiable, models.length);
      modelSettings = new CRDTYOLOv8ModelParameters[models.length];
      for (int i = 0; i < models.length; ++i)
      {
         modelSettings[i] = new CRDTYOLOv8ModelParameters(crdtInfo, models[i]);
      }
   }

   public CRDTBidirectionalStringArray getModelsToRun()
   {
      return modelsToRun;
   }

   public CRDTYOLOv8ModelParameters[] getModelSettings()
   {
      return modelSettings;
   }

   public void toMessage(YOLOv8ExecutorSettings messageToPack)
   {
      latestTimestampModifiable.toMessage(messageToPack.getLatestTimestampModifiable());

      modelsToRun.toMessage(messageToPack.getModelsToRun());

      messageToPack.getModelsToRun().clear();
      for (CRDTYOLOv8ModelParameters modelSetting : modelSettings)
      {
         modelSetting.toMessage(messageToPack.getModelSettings().add());
      }
   }

   public void fromMessage(YOLOv8ExecutorSettings message)
   {
      latestTimestampModifiable.fromMessage(message.getLatestTimestampModifiable());

      modelsToRun.fromMessage(message.getModelsToRun());

      for (int i = 0; i < message.getModelSettings().size() && i < modelSettings.length; ++i)
      {
         modelSettings[i].fromMessage(message.getModelSettings().get(i));
      }
   }
}

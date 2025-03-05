package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ModelInfo;
import perception_msgs.msg.dds.YOLOv8ModelSettings;
import us.ihmc.communication.crdt.CRDTBidirectionalBooleanArray;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalFloatArray;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerArray;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

public class CRDTYOLOv8ModelParameters //extends LatestTimestampModifiable
{
   private final LatestTimestampModifiable latestTimestampModifiable;

   private final YOLOv8ModelInfo modelInfo;

   // YOLO model parameters
   private final CRDTBidirectionalBooleanArray ignoredObjectClasses;
   private final CRDTBidirectionalFloatArray confidenceThresholds;
   private final CRDTBidirectionalFloatArray maskThresholds;
   private final CRDTBidirectionalFloat nmsThreshold;

   // Output processing parameters
   private final CRDTBidirectionalIntegerArray erosionKernelRadii;
   private final CRDTBidirectionalFloatArray outlierThresholds;

   public CRDTYOLOv8ModelParameters(LatestTimestampModifiable latestTimestampModifiable, YOLOv8ModelInfo modelInfo)
   {
      this.latestTimestampModifiable = latestTimestampModifiable;

      this.modelInfo = modelInfo;

      int objectClassCount = modelInfo.getDetectableObjectClasses().size();
      ignoredObjectClasses = new CRDTBidirectionalBooleanArray(latestTimestampModifiable, objectClassCount);
      confidenceThresholds = new CRDTBidirectionalFloatArray(latestTimestampModifiable, objectClassCount);
      maskThresholds = new CRDTBidirectionalFloatArray(latestTimestampModifiable, objectClassCount);
      nmsThreshold = new CRDTBidirectionalFloat(latestTimestampModifiable, 0.2f);

      erosionKernelRadii = new CRDTBidirectionalIntegerArray(latestTimestampModifiable, objectClassCount);
      outlierThresholds = new CRDTBidirectionalFloatArray(latestTimestampModifiable, objectClassCount);
   }

   public void applyToModel(YOLOv8Model model)
   {
      if (!model.getName().equals(getModelName()))
         throw new IllegalArgumentException("Attempting to apply settings for the wrong model");

      model.setIgnoredClasses(ignoredObjectClasses.getValue());
      model.setConfidenceThresholds(confidenceThresholds.getValue());
      model.setMaskThresholds(maskThresholds.getValue());
      model.setNMSThreshold(nmsThreshold.getValue());
   }

   public boolean isModified()
   {
      return latestTimestampModifiable.isModified();
   }

   public String getModelName()
   {
      return modelInfo.getModelNameAsString();
   }

   public String[] getDetectableObjectClasses()
   {
      return modelInfo.getDetectableObjectClasses().toStringArray();
   }

   public CRDTBidirectionalBooleanArray getIgnoredObjectClasses()
   {
      return ignoredObjectClasses;
   }

   public CRDTBidirectionalFloatArray getConfidenceThresholds()
   {
      return confidenceThresholds;
   }

   public CRDTBidirectionalFloatArray getMaskThresholds()
   {
      return maskThresholds;
   }

   public CRDTBidirectionalFloat getNMSThreshold()
   {
      return nmsThreshold;
   }

   public CRDTBidirectionalIntegerArray getErosionKernelRadii()
   {
      return erosionKernelRadii;
   }

   public CRDTBidirectionalFloatArray getOutlierThresholds()
   {
      return outlierThresholds;
   }

   public void toMessage(YOLOv8ModelSettings messageToPack)
   {
//      latestTimestampModifiable.toMessage(messageToPack.getLatestTimestampModifiable());

      ignoredObjectClasses.toMessage(messageToPack.getIgnoredObjectClasses());
      confidenceThresholds.toMessage(messageToPack.getConfidenceThresholds().toArray());
      maskThresholds.toMessage(messageToPack.getMaskThresholds());
      messageToPack.setNonMaximumSuppressionThreshold(nmsThreshold.toMessage());

      erosionKernelRadii.toMessage(messageToPack.getErosionKernelRadii());
      outlierThresholds.toMessage(messageToPack.getOutlierThresholds());
   }

   public void fromMessage(YOLOv8ModelSettings message)
   {
//      fromMessage(message.getLatestTimestampModifiable());
//
      ignoredObjectClasses.fromMessage(message.getIgnoredObjectClasses());
      confidenceThresholds.fromMessage(message.getConfidenceThresholds().toArray());
      maskThresholds.fromMessage(message.getMaskThresholds());
      nmsThreshold.fromMessage(message.getNonMaximumSuppressionThreshold());

      erosionKernelRadii.fromMessage(message.getErosionKernelRadii());
      outlierThresholds.fromMessage(message.getOutlierThresholds());
   }
}

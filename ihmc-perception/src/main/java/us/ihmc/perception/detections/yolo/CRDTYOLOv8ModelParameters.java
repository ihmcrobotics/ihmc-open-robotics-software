package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ModelInfo;
import perception_msgs.msg.dds.YOLOv8ModelSettings;
import us.ihmc.communication.crdt.CRDTBidirectionalBooleanArray;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalFloatArray;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerArray;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

import java.util.Arrays;

public class CRDTYOLOv8ModelParameters
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
      Arrays.fill(ignoredObjectClasses.getValueAndModify(), false);
      confidenceThresholds = new CRDTBidirectionalFloatArray(latestTimestampModifiable, objectClassCount);
      Arrays.fill(confidenceThresholds.getValueAndModify(), 0.7f);
      maskThresholds = new CRDTBidirectionalFloatArray(latestTimestampModifiable, objectClassCount);
      Arrays.fill(maskThresholds.getValueAndModify(), 0.0f);
      nmsThreshold = new CRDTBidirectionalFloat(latestTimestampModifiable, 0.1f);

      erosionKernelRadii = new CRDTBidirectionalIntegerArray(latestTimestampModifiable, objectClassCount);
      Arrays.fill(erosionKernelRadii.getValueAndModify(), 1);
      outlierThresholds = new CRDTBidirectionalFloatArray(latestTimestampModifiable, objectClassCount);
      Arrays.fill(outlierThresholds.getValueAndModify(), 1.0f);
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
      messageToPack.setModelName(getModelName());

      ignoredObjectClasses.toMessage(messageToPack.getIgnoredObjectClasses());
      confidenceThresholds.toMessage(messageToPack.getConfidenceThresholds());
      maskThresholds.toMessage(messageToPack.getMaskThresholds());
      messageToPack.setNonMaximumSuppressionThreshold(nmsThreshold.toMessage());

      erosionKernelRadii.toMessage(messageToPack.getErosionKernelRadii());
      outlierThresholds.toMessage(messageToPack.getOutlierThresholds());
   }

   public void fromMessage(YOLOv8ModelSettings message)
   {
      ignoredObjectClasses.fromMessage(message.getIgnoredObjectClasses());
      confidenceThresholds.fromMessage(message.getConfidenceThresholds());
      maskThresholds.fromMessage(message.getMaskThresholds());
      nmsThreshold.fromMessage(message.getNonMaximumSuppressionThreshold());

      erosionKernelRadii.fromMessage(message.getErosionKernelRadii());
      outlierThresholds.fromMessage(message.getOutlierThresholds());
   }
}

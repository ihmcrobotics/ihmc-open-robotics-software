package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ModelParameters;
import us.ihmc.communication.crdt.CRDTBidirectionalBooleanArray;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalFloatArray;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerArray;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

import java.util.Arrays;

public class SyncedYOLOv8ModelParameters extends LatestTimestampModifiable
{
   private static final int MAX_DETECTABLE_OBJECTS = 96;

   // YOLO model parameters
   private final CRDTBidirectionalBooleanArray ignoredObjectClasses;
   private final CRDTBidirectionalFloatArray confidenceThresholds;
   private final CRDTBidirectionalFloatArray maskThresholds;
   private final CRDTBidirectionalFloat nmsThreshold;

   // Output processing parameters
   private final CRDTBidirectionalIntegerArray erosionKernelRadii;
   private final CRDTBidirectionalFloatArray outlierThresholds;

   public SyncedYOLOv8ModelParameters(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
      setModifierName(getClass().getSimpleName());

      ignoredObjectClasses = new CRDTBidirectionalBooleanArray(this, SyncedYOLOv8ModelParameters.MAX_DETECTABLE_OBJECTS);
      Arrays.fill(ignoredObjectClasses.getValue(), false);
      confidenceThresholds = new CRDTBidirectionalFloatArray(this, SyncedYOLOv8ModelParameters.MAX_DETECTABLE_OBJECTS);
      Arrays.fill(confidenceThresholds.getValue(), 0.7f);
      maskThresholds = new CRDTBidirectionalFloatArray(this, SyncedYOLOv8ModelParameters.MAX_DETECTABLE_OBJECTS);
      Arrays.fill(maskThresholds.getValue(), 0.0f);
      nmsThreshold = new CRDTBidirectionalFloat(this, 0.1f);

      erosionKernelRadii = new CRDTBidirectionalIntegerArray(this, SyncedYOLOv8ModelParameters.MAX_DETECTABLE_OBJECTS);
      Arrays.fill(erosionKernelRadii.getValue(), 1);
      outlierThresholds = new CRDTBidirectionalFloatArray(this, SyncedYOLOv8ModelParameters.MAX_DETECTABLE_OBJECTS);
      Arrays.fill(outlierThresholds.getValue(), 2.0f);
   }

   /**
    * Applies the current parameter values to the passed in model.
    * The past in model must match the model info used to construct the parameters.
    *
    * @param model YOLO model to which parameters will be applied.
    */
   public void applyToModel(YOLOv8Model model)
   {
      if (model == null)
         return;

      model.setIgnoredClasses(ignoredObjectClasses.getValue());
      model.setConfidenceThresholds(confidenceThresholds.getValue());
      model.setMaskThresholds(maskThresholds.getValue());
      model.setNMSThreshold(nmsThreshold.getValue());
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

   public void toMessage(YOLOv8ModelParameters messageToPack)
   {
      toMessage(messageToPack.getLatestTimestampModifiable());

      ignoredObjectClasses.toMessage(messageToPack.getIgnoredObjectClasses());
      confidenceThresholds.toMessage(messageToPack.getConfidenceThresholds());
      maskThresholds.toMessage(messageToPack.getMaskThresholds());
      messageToPack.setNonMaximumSuppressionThreshold(nmsThreshold.toMessage());

      erosionKernelRadii.toMessage(messageToPack.getErosionKernelRadii());
      outlierThresholds.toMessage(messageToPack.getOutlierThresholds());
   }

   public void fromMessage(YOLOv8ModelParameters message)
   {
      fromMessage(message.getLatestTimestampModifiable());

      ignoredObjectClasses.fromMessage(message.getIgnoredObjectClasses());
      confidenceThresholds.fromMessage(message.getConfidenceThresholds());
      maskThresholds.fromMessage(message.getMaskThresholds());
      nmsThreshold.fromMessage(message.getNonMaximumSuppressionThreshold());

      erosionKernelRadii.fromMessage(message.getErosionKernelRadii());
      outlierThresholds.fromMessage(message.getOutlierThresholds());
   }
}

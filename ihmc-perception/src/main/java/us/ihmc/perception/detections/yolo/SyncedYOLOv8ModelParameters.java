package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.YOLOv8ModelInfo;
import perception_msgs.msg.dds.YOLOv8ModelParameters;
import us.ihmc.communication.crdt.CRDTBidirectionalBooleanArray;
import us.ihmc.communication.crdt.CRDTBidirectionalFloat;
import us.ihmc.communication.crdt.CRDTBidirectionalFloatArray;
import us.ihmc.communication.crdt.CRDTBidirectionalIntegerArray;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;

import java.util.Arrays;

public class SyncedYOLOv8ModelParameters
{
   private final YOLOv8ModelInfo modelInfo;

   // YOLO model parameters
   private final CRDTBidirectionalBooleanArray ignoredObjectClasses;
   private final CRDTBidirectionalFloatArray confidenceThresholds;
   private final CRDTBidirectionalFloatArray maskThresholds;
   private final CRDTBidirectionalFloat nmsThreshold;

   // Output processing parameters
   private final CRDTBidirectionalIntegerArray erosionKernelRadii;
   private final CRDTBidirectionalFloatArray outlierThresholds;

   public SyncedYOLOv8ModelParameters(LatestTimestampModifiable executorParameters, YOLOv8ModelInfo modelInfo)
   {
      this.modelInfo = modelInfo;

      int objectClassCount = modelInfo.getDetectableObjectClasses().size();
      ignoredObjectClasses = new CRDTBidirectionalBooleanArray(executorParameters, objectClassCount);
      Arrays.fill(ignoredObjectClasses.getValue(), false);
      confidenceThresholds = new CRDTBidirectionalFloatArray(executorParameters, objectClassCount);
      Arrays.fill(confidenceThresholds.getValue(), 0.7f);
      maskThresholds = new CRDTBidirectionalFloatArray(executorParameters, objectClassCount);
      Arrays.fill(maskThresholds.getValue(), 0.0f);
      nmsThreshold = new CRDTBidirectionalFloat(executorParameters, 0.1f);

      erosionKernelRadii = new CRDTBidirectionalIntegerArray(executorParameters, objectClassCount);
      Arrays.fill(erosionKernelRadii.getValue(), 1);
      outlierThresholds = new CRDTBidirectionalFloatArray(executorParameters, objectClassCount);
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
      if (!model.getName().equals(getModelName()))
         throw new IllegalArgumentException("Attempting to apply settings for the wrong model");

      model.setIgnoredClasses(ignoredObjectClasses.getValue());
      model.setConfidenceThresholds(confidenceThresholds.getValue());
      model.setMaskThresholds(maskThresholds.getValue());
      model.setNMSThreshold(nmsThreshold.getValue());
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

   public void toMessage(YOLOv8ModelParameters messageToPack)
   {
      messageToPack.setModelName(getModelName());

      ignoredObjectClasses.toMessage(messageToPack.getIgnoredObjectClasses());
      confidenceThresholds.toMessage(messageToPack.getConfidenceThresholds());
      maskThresholds.toMessage(messageToPack.getMaskThresholds());
      messageToPack.setNonMaximumSuppressionThreshold(nmsThreshold.toMessage());

      erosionKernelRadii.toMessage(messageToPack.getErosionKernelRadii());
      outlierThresholds.toMessage(messageToPack.getOutlierThresholds());
   }

   public void fromMessage(YOLOv8ModelParameters message)
   {
      ignoredObjectClasses.fromMessage(message.getIgnoredObjectClasses());
      confidenceThresholds.fromMessage(message.getConfidenceThresholds());
      maskThresholds.fromMessage(message.getMaskThresholds());
      nmsThreshold.fromMessage(message.getNonMaximumSuppressionThreshold());

      erosionKernelRadii.fromMessage(message.getErosionKernelRadii());
      outlierThresholds.fromMessage(message.getOutlierThresholds());
   }

   public void disableAllClasses()
   {
      for (int i = 0; i < ignoredObjectClasses.getLength(); i++)
         ignoredObjectClasses.setValue(i, true);
   }

   public void enableClass(String className, float confidenceThreshold)
   {
      String[] detectableClasses = getDetectableObjectClasses();
      for (int i = 0; i < detectableClasses.length; i++)
      {
         if (detectableClasses[i].equalsIgnoreCase(className))
         {
            ignoredObjectClasses.setValue(i, false);
            confidenceThresholds.setValue(i, confidenceThreshold);
            break;
         }
      }
   }
}
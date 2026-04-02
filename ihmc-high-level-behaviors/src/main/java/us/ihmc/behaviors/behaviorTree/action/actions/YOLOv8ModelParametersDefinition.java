package us.ihmc.behaviors.behaviorTree.action.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ModelParameters;

public class YOLOv8ModelParametersDefinition extends SyncedYOLOv8ModelParameters
{
   private final boolean[] onDiskIgnoredObjectClasses;
   private final float[] onDiskConfidenceThresholds;
   private final float[] onDiskMaskThresholds;
   private float onDiskNMSThreshold;
   private final int[] onDiskErosionKernelRadii;
   private final float[] onDiskOutlierThresholds;
   private boolean undoAvailable = false;

   public YOLOv8ModelParametersDefinition(LatestTimestampModifiable latestTimestampModifiable, YOLOv8ModelInfo modelInfo)
   {
      super(latestTimestampModifiable, modelInfo);

      onDiskIgnoredObjectClasses = new boolean[ignoredObjectClasses.getLength()];
      onDiskConfidenceThresholds = new float[confidenceThresholds.getLength()];
      onDiskMaskThresholds = new float[maskThresholds.getLength()];
      onDiskErosionKernelRadii = new int[erosionKernelRadii.getLength()];
      onDiskOutlierThresholds = new float[outlierThresholds.getLength()];
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      ArrayNode ignoredObjectClassesNode = jsonNode.putArray("ignoredObjectClasses");
      for (int i = 0; i < ignoredObjectClasses.getLength(); i++)
         ignoredObjectClassesNode.add(ignoredObjectClasses.getValueReadOnly(i));

      ArrayNode confidenceThresholdsNode = jsonNode.putArray("confidenceThresholds");
      for (int i = 0; i < confidenceThresholds.getLength(); i++)
         confidenceThresholdsNode.add(confidenceThresholds.getValueReadOnly(i));

      ArrayNode maskThresholdsNode = jsonNode.putArray("maskThresholds");
      for (int i = 0; i < maskThresholds.getLength(); i++)
         maskThresholdsNode.add(maskThresholds.getValueReadOnly(i));

      jsonNode.put("nmsThreshold", nmsThreshold.getValue());

      ArrayNode erosionKernelRadiiNode = jsonNode.putArray("erosionKernelRadii");
      for (int i = 0; i < erosionKernelRadii.getLength(); i++)
         erosionKernelRadiiNode.add(erosionKernelRadii.getValueReadOnly(i));

      ArrayNode outlierThresholdsNode = jsonNode.putArray("outlierThresholds");
      for (int i = 0; i < outlierThresholds.getLength(); i++)
         outlierThresholdsNode.add(outlierThresholds.getValueReadOnly(i));
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      if (jsonNode.get("ignoredObjectClasses") instanceof ArrayNode ignoredObjectClassesNode)
      {
         for (int i = 0; i < ignoredObjectClasses.getLength() && i < ignoredObjectClassesNode.size(); i++)
            ignoredObjectClasses.setValue(i, ignoredObjectClassesNode.get(i).asBoolean());
      }

      if (jsonNode.get("confidenceThresholds") instanceof ArrayNode confidenceThresholdsNode)
      {
         for (int i = 0; i < confidenceThresholds.getLength() && i < confidenceThresholdsNode.size(); i++)
            confidenceThresholds.setValue(i, (float) confidenceThresholdsNode.get(i).asDouble());
      }

      if (jsonNode.get("maskThresholds") instanceof ArrayNode maskThresholdsNode)
      {
         for (int i = 0; i < maskThresholds.getLength() && i < maskThresholdsNode.size(); i++)
            maskThresholds.setValue(i, (float) maskThresholdsNode.get(i).asDouble());
      }

      if (jsonNode.has("nmsThreshold"))
         nmsThreshold.setValue((float) jsonNode.get("nmsThreshold").asDouble());

      if (jsonNode.get("erosionKernelRadii") instanceof ArrayNode erosionKernelRadiiNode)
      {
         for (int i = 0; i < erosionKernelRadii.getLength() && i < erosionKernelRadiiNode.size(); i++)
            erosionKernelRadii.setValue(i, erosionKernelRadiiNode.get(i).asInt());
      }

      if (jsonNode.get("outlierThresholds") instanceof ArrayNode outlierThresholdsNode)
      {
         for (int i = 0; i < outlierThresholds.getLength() && i < outlierThresholdsNode.size(); i++)
            outlierThresholds.setValue(i, (float) outlierThresholdsNode.get(i).asDouble());
      }
   }

   public void setOnDiskFields()
   {
      for (int i = 0; i < ignoredObjectClasses.getLength(); i++)
         onDiskIgnoredObjectClasses[i] = ignoredObjectClasses.getValueReadOnly(i);

      for (int i = 0; i < confidenceThresholds.getLength(); i++)
         onDiskConfidenceThresholds[i] = confidenceThresholds.getValueReadOnly(i);

      for (int i = 0; i < maskThresholds.getLength(); i++)
         onDiskMaskThresholds[i] = maskThresholds.getValueReadOnly(i);

      onDiskNMSThreshold = nmsThreshold.getValue();

      for (int i = 0; i < erosionKernelRadii.getLength(); i++)
         onDiskErosionKernelRadii[i] = erosionKernelRadii.getValueReadOnly(i);

      for (int i = 0; i < outlierThresholds.getLength(); i++)
         onDiskOutlierThresholds[i] = outlierThresholds.getValueReadOnly(i);

      undoAvailable = true;
   }

   public void undoAllNontopologicalChanges()
   {
      if (!undoAvailable)
         return;

      for (int i = 0; i < ignoredObjectClasses.getLength(); i++)
         ignoredObjectClasses.setValue(i, onDiskIgnoredObjectClasses[i]);

      for (int i = 0; i < confidenceThresholds.getLength(); i++)
         confidenceThresholds.setValue(i, onDiskConfidenceThresholds[i]);

      for (int i = 0; i < maskThresholds.getLength(); i++)
         maskThresholds.setValue(i, onDiskMaskThresholds[i]);

      nmsThreshold.setValue(onDiskNMSThreshold);

      for (int i = 0; i < erosionKernelRadii.getLength(); i++)
         erosionKernelRadii.setValue(i, onDiskErosionKernelRadii[i]);

      for (int i = 0; i < outlierThresholds.getLength(); i++)
         outlierThresholds.setValue(i, onDiskOutlierThresholds[i]);
   }

   public boolean hasChanges()
   {
      if (!undoAvailable)
         return false;

      for (int i = 0; i < ignoredObjectClasses.getLength(); i++)
         if (ignoredObjectClasses.getValueReadOnly(i) != onDiskIgnoredObjectClasses[i])
            return true;

      for (int i = 0; i < confidenceThresholds.getLength(); i++)
         if (confidenceThresholds.getValueReadOnly(i) != onDiskConfidenceThresholds[i])
            return true;

      for (int i = 0; i < maskThresholds.getLength(); i++)
         if (maskThresholds.getValueReadOnly(i) != onDiskMaskThresholds[i])
            return true;

      if (nmsThreshold.getValue() != onDiskNMSThreshold)
         return true;

      for (int i = 0; i < erosionKernelRadii.getLength(); i++)
         if (erosionKernelRadii.getValueReadOnly(i) != onDiskErosionKernelRadii[i])
            return true;

      for (int i = 0; i < outlierThresholds.getLength(); i++)
         if (outlierThresholds.getValueReadOnly(i) != onDiskOutlierThresholds[i])
            return true;

      return false;
   }
}

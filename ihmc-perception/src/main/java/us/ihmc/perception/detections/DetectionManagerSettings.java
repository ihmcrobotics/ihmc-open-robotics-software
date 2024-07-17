package us.ihmc.perception.detections;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class DetectionManagerSettings extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey poseFilterAlpha = keys.addDoubleKey("Pose Filter Alpha", 0.1);
   public static final DoubleStoredPropertyKey maxMatchDistanceSquared = keys.addDoubleKey("Max Match Distance Squared", 1.0);
   public static final DoubleStoredPropertyKey acceptanceAverageConfidence = keys.addDoubleKey("Acceptance Average Confidence", 0.85);
   public static final DoubleStoredPropertyKey stabilityAverageConfidence = keys.addDoubleKey("Stability Average Confidence", 0.85);
   public static final DoubleStoredPropertyKey stabilityDetectionFrequency = keys.addDoubleKey("Stability Detection Frequency", 5.0);
   public static final DoubleStoredPropertyKey detectionHistoryDuration = keys.addDoubleKey("Detection History Duration", 1.0);

   public DetectionManagerSettings()
   {
      super(keys, DetectionManagerSettings.class);
   }

   public double getPoseFilterAlpha()
   {
      return get(poseFilterAlpha);
   }

   public double getMaxMatchDistanceSquared()
   {
      return get(maxMatchDistanceSquared);
   }

   public double getAcceptanceAverageConfidence()
   {
      return get(acceptanceAverageConfidence);
   }

   public double getStabilityAverageConfidence()
   {
      return get(stabilityAverageConfidence);
   }

   public double getStabilityDetectionFrequency()
   {
      return get(stabilityDetectionFrequency);
   }

   public double getDetectionHistoryDuration()
   {
      return get(detectionHistoryDuration);
   }

   public void setPoseFilterAlpha(double poseFilterAlpha)
   {
      set(DetectionManagerSettings.poseFilterAlpha, poseFilterAlpha);
   }

   public void setMaxMatchDistanceSquared(double maxMatchDistanceSquared)
   {
      set(DetectionManagerSettings.maxMatchDistanceSquared, maxMatchDistanceSquared);
   }

   public void setAcceptanceAverageConfidence(double acceptanceAverageConfidence)
   {
      set(DetectionManagerSettings.acceptanceAverageConfidence, acceptanceAverageConfidence);
   }

   public void setStabilityAverageConfidence(double stabilityAverageConfidence)
   {
      set(DetectionManagerSettings.stabilityAverageConfidence, stabilityAverageConfidence);
   }

   public void setStabilityDetectionFrequency(double stabilityDetectionFrequency)
   {
      set(DetectionManagerSettings.stabilityDetectionFrequency, stabilityDetectionFrequency);
   }

   public void setDetectionHistoryDuration(double detectionHistoryDuration)
   {
      set(DetectionManagerSettings.detectionHistoryDuration, detectionHistoryDuration);
   }
}

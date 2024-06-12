package us.ihmc.perception.detections;

import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.Comparator;
import java.util.TreeSet;

public class PersistentDetection<T extends InstantDetection>
{
   // The first element of this set is the oldest detection, and the last element is the most recent detection
   private final TreeSet<T> detectionHistory = new TreeSet<>(Comparator.comparing(InstantDetection::getDetectionTime));
   private Duration historyDuration;

   private String detectionClass = null;

   // If the total confidence per second is greater than this value, the detections will be considered stable.
   private double stabilityThreshold;

   public PersistentDetection(double stabilityThreshold)
   {
      this(stabilityThreshold, Duration.ofSeconds(1L));
   }

   public PersistentDetection(double stabilityThreshold, double historyDurationSeconds)
   {
      this(stabilityThreshold, TimeTools.durationOfSeconds(historyDurationSeconds));
   }

   public PersistentDetection(double stabilityThreshold, Duration historyDuration)
   {
      setStabilityThreshold(stabilityThreshold);
      setHistoryLength(historyDuration);
   }

   /**
    * Add a new instant detection to the history of detections
    * @param newDetection A new {@link us.ihmc.perception.detections.InstantDetection} of the same class as the first detection
    *                     added to this {@link us.ihmc.perception.detections.PersistentDetection}
    */
   public void addDetection(T newDetection)
   {
      // Record the detection class of the first detection added
      if (detectionClass == null)
         detectionClass = newDetection.getDetectionClass();

      // ensure only detection of the same class are added to the history
      if (!newDetection.getDetectionClass().equals(detectionClass))
         throw new IllegalArgumentException(String.format("New detection's class (%s) does not match original detection's class (%s)",
                                                          newDetection.getDetectionClass(),
                                                          detectionClass));

      detectionHistory.add(newDetection);
   }

   /**
    * @return The most recent {@link us.ihmc.perception.detections.InstantDetection} added to the history,
    * based on the detection's {@link java.time.Instant}.
    * Null if there are no detection in the history, or if the most recent detection is older than the history duration.
    */
   public InstantDetection getMostRecentDetection()
   {
      updateHistory();
      if (detectionHistory.isEmpty())
         return null;

      return detectionHistory.last();
   }

   /**
    * Set the duration of detection history stored by this object.
    * {@link us.ihmc.perception.detections.InstantDetection} objects older than this duration will be removed from the history.
    * @param historyDurationSeconds Number of seconds of history to store. Must be positive.
    */
   public void setHistoryLength(double historyDurationSeconds)
   {
      setHistoryLength(TimeTools.durationOfSeconds(historyDurationSeconds));
   }

   /**
    * Set the duration of detection history stored by this object.
    * {@link us.ihmc.perception.detections.InstantDetection} objects older than this duration will be removed from the history.
    * @param historyDuration Duration of history to store. Must be positive.
    */
   public void setHistoryLength(Duration historyDuration)
   {
      if (historyDuration.isNegative())
         throw new IllegalArgumentException("History duration must be a positive duration");

      this.historyDuration = historyDuration;
   }

   public void setStabilityThreshold(double stabilityThreshold)
   {
      this.stabilityThreshold = Math.abs(stabilityThreshold);
   }

   public boolean isStable()
   {
      return getConfidencePerSecond() > stabilityThreshold;
   }

   public double getConfidencePerSecond()
   {
      return getConfidencePerSecond(Instant.now());
   }

   public double getConfidencePerSecond(Instant now)
   {
      updateHistory(now);

      double confidenceSum = 0.0;
      for (InstantDetection detection : detectionHistory) {
         confidenceSum += detection.getConfidence();
      }

      double historyDurationSeconds = TimeTools.toDoubleSeconds(historyDuration);
      return confidenceSum / historyDurationSeconds;
   }

   // This method should be called before accessing history...
   private void updateHistory()
   {
      updateHistory(Instant.now());
   }

   // Or this method works too!
   private void updateHistory(Instant now)
   {
      // Remove detections that are too old
      Instant beginningOfHistory = now.minus(historyDuration);
      detectionHistory.removeIf(detection -> detection.getDetectionTime().isBefore(beginningOfHistory));
   }
}

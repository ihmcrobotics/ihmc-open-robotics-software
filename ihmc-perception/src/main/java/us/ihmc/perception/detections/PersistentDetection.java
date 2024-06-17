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

   private final Class<?> instantDetectionClass;
   private final String detectedObjectClass;
   private final String detectedObjectName;

   // If the total confidence per second is greater than this value, the detections will be considered stable.
   private double stabilityConfidenceThreshold;
   private int stabilityMinHistorySize;

   public PersistentDetection(T firstDetection)
   {
      this(firstDetection, 0.5, 3);
   }

   public PersistentDetection(T firstDetection, double stabilityThreshold, int stabilityMinHistorySize)
   {
      this(firstDetection, stabilityThreshold, stabilityMinHistorySize, Duration.ofSeconds(1L));
   }

   public PersistentDetection(T firstDetection, double stabilityConfidenceThreshold, int stabilityMinHistorySize, double historyDurationSeconds)
   {
      this(firstDetection, stabilityConfidenceThreshold, stabilityMinHistorySize, TimeTools.durationOfSeconds(historyDurationSeconds));
   }

   public PersistentDetection(T firstDetection, double stabilityConfidenceThreshold, int stabilityMinHistorySize, Duration historyDuration)
   {
      instantDetectionClass = firstDetection.getClass();
      detectedObjectClass = firstDetection.getDetectedObjectClass();
      detectedObjectName = firstDetection.getDetectedObjectName();

      addDetection(firstDetection);
      setStabilityConfidenceThreshold(stabilityConfidenceThreshold);
      setStabilityMinHistorySize(stabilityMinHistorySize);
      setHistoryDuration(historyDuration);
   }

   /**
    * Add a new instant detection to the history of detections
    * @param newDetection A new {@link InstantDetection} of the same class as the first detection
    *                     added to this {@link PersistentDetection}
    */
   public void addDetection(T newDetection)
   {
      // ensure only detection of the same class are added to the history
      if (!newDetection.getDetectedObjectClass().equals(detectedObjectClass))
         throw new IllegalArgumentException(String.format("New detection's class (%s) does not match original detection's class (%s)",
                                                          newDetection.getDetectedObjectClass(), detectedObjectClass));

      detectionHistory.add(newDetection);
   }

   /**
    * @return The most recent {@link InstantDetection} added to the history,
    * based on the detection's {@link java.time.Instant}.
    */
   public T getMostRecentDetection()
   {
      return detectionHistory.last();
   }

   public Class<?> getInstantDetectionClass()
   {
      return instantDetectionClass;
   }

   public String getDetectedObjectClass()
   {
      return detectedObjectClass;
   }

   public String getDetectedObjectName()
   {
      return detectedObjectName;
   }

   public int getHistorySize()
   {
      return detectionHistory.size();
   }

   /**
    * Set the duration of detection history stored by this object.
    * {@link InstantDetection} objects older than this duration will be removed from the history.
    * @param historyDurationSeconds Number of seconds of history to store. Must be positive.
    */
   public void setHistoryDuration(double historyDurationSeconds)
   {
      setHistoryDuration(TimeTools.durationOfSeconds(historyDurationSeconds));
   }

   /**
    * Set the duration of detection history stored by this object.
    * {@link InstantDetection} objects older than this duration will be removed from the history when {@link #updateHistory()} is called.
    * @param historyDuration Duration of history to store. Must be positive.
    */
   public void setHistoryDuration(Duration historyDuration)
   {
      if (historyDuration.isNegative())
         throw new IllegalArgumentException("History duration must be a positive duration");

      this.historyDuration = historyDuration;
   }

   public void setStabilityConfidenceThreshold(double stabilityConfidenceThreshold)
   {
      if (stabilityConfidenceThreshold < 0.0 || stabilityConfidenceThreshold > 1.0)
         throw new IllegalArgumentException("Stability threshold must be between 0.0 and 1.0 (inclusive)");

      this.stabilityConfidenceThreshold = stabilityConfidenceThreshold;
   }

   public void setStabilityMinHistorySize(int minHistoryLength)
   {
      this.stabilityMinHistorySize = minHistoryLength;
   }

   public boolean isStable()
   {
      return getHistorySize() >= stabilityMinHistorySize && getAverageConfidence() > stabilityConfidenceThreshold;
   }

   /**
    * Calculates the average confidence of the {@link InstantDetection}s in the stored history.
    * {@link #updateHistory()} should be called before this method to get the up-to-date value!
    * @return Average confidence of {@link InstantDetection}s in history
    */
   public double getAverageConfidence()
   {
      double confidenceSum = 0.0;
      for (T detection : detectionHistory) {
         confidenceSum += detection.getConfidence();
      }

      return confidenceSum / detectionHistory.size();
   }

   /**
    * Clears history items older than {@code historyDuration}, except for the most recent detection.
    */
   public void updateHistory()
   {
      updateHistory(Instant.now());
   }

   /**
    * Clears history items older than {@code now - historyDuration}, except for the most recent detection.
    * @param now Present time instant.
    */
   public void updateHistory(Instant now)
   {
      // Remove detections that are too old
      detectionHistory.removeIf(detection -> detectionExpired(detection, now) && !detection.equals(getMostRecentDetection()));
   }

   private boolean detectionExpired(InstantDetection detection, Instant now)
   {
      return detection.getDetectionTime().isBefore(now.minus(historyDuration));
   }
}

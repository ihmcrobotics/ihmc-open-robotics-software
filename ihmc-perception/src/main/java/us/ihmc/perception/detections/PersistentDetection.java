package us.ihmc.perception.detections;

import us.ihmc.commons.Conversions;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;

public class PersistentDetection<T extends InstantDetection>
{
   // The first element of this set is the oldest detection, and the last element is the most recent detection
   private final SortedSet<T> detectionHistory = new TreeSet<>(Comparator.comparing(InstantDetection::getDetectionTime));
   private Duration historyDuration;

   private final InstantDetection firstDetection;

   private double stabilityConfidenceThreshold;
   private double stabilityDetectionFrequency;

   private boolean readyForDeletion = false;

   public PersistentDetection(T firstDetection)
   {
      this(firstDetection, 0.5, 5.0, 1.0);
   }

   public PersistentDetection(T firstDetection, double stabilityConfidenceThreshold, double stabilityDetectionFrequency, double historyDurationSeconds)
   {
      this(firstDetection, stabilityConfidenceThreshold, stabilityDetectionFrequency, TimeTools.durationOfSeconds(historyDurationSeconds));
   }

   public PersistentDetection(T firstDetection, double stabilityConfidenceThreshold, double stabilityDetectionFrequency, Duration historyDuration)
   {
      this.firstDetection = firstDetection;

      addDetection(firstDetection);
      setStabilityConfidenceThreshold(stabilityConfidenceThreshold);
      setStabilityDetectionFrequency(stabilityDetectionFrequency);
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
      if (!newDetection.getDetectedObjectClass().equals(getDetectedObjectClass()))
         throw new IllegalArgumentException(String.format("New detection's class (%s) does not match original detection's class (%s)",
                                                          newDetection.getDetectedObjectClass(), getDetectedObjectClass()));

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
      return getMostRecentDetection().getClass();
   }

   public String getDetectedObjectClass()
   {
      return firstDetection.getDetectedObjectClass();
   }

   public String getDetectedObjectName()
   {
      return firstDetection
            .getDetectedObjectName();
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
      this.stabilityConfidenceThreshold = stabilityConfidenceThreshold;
   }

   public void setStabilityDetectionFrequency(double stabilityDetectionFrequency)
   {
      this.stabilityDetectionFrequency = stabilityDetectionFrequency;
   }

   public boolean isOldEnough()
   {
      return isOldEnough(Instant.now());
   }

   public boolean isOldEnough(Instant now)
   {
      return firstDetection.getDetectionTime().isBefore(now.minus(historyDuration));
   }

   public boolean isStable()
   {
      return isStable(Instant.now());
   }

   public boolean isStable(Instant now)
   {
      return getDetectionFrequencyDecaying(now) > stabilityDetectionFrequency && getAverageConfidence() > stabilityConfidenceThreshold;
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

   public double getDetectionFrequency()
   {
      if (detectionHistory.size() <= 1)
         return 0.0;

      Instant oldestDetectionTime = detectionHistory.first().getDetectionTime();
      Instant newestDetectionTime = detectionHistory.last().getDetectionTime();

      long periodNanos = oldestDetectionTime.until(newestDetectionTime, ChronoUnit.NANOS);
      return (getHistorySize() - 1.0) / Conversions.nanosecondsToSeconds(periodNanos);
   }

   public double getDetectionFrequencyDecaying()
   {
      return getDetectionFrequencyDecaying(Instant.now());
   }

   public double getDetectionFrequencyDecaying(Instant now)
   {
      Instant oldestDetectionTime = detectionHistory.first().getDetectionTime();

      long periodNanos = oldestDetectionTime.until(now, ChronoUnit.NANOS);
      return getHistorySize() / Conversions.nanosecondsToSeconds(periodNanos);
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

   public SortedSet<T> getDetectionHistory()
   {
      return detectionHistory;
   }

   public void markForDeletion()
   {
      readyForDeletion = true;
   }

   public boolean isReadyForDeletion()
   {
      return readyForDeletion;
   }

   private boolean detectionExpired(InstantDetection detection, Instant now)
   {
      return detection.getDetectionTime().isBefore(now.minus(historyDuration));
   }
}

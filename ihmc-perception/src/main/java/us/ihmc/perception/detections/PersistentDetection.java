package us.ihmc.perception.detections;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.UUID;

public class PersistentDetection
{
   // The first element of this set is the oldest detection, and the last element is the most recent detection
   private final SortedSet<InstantDetection> detectionHistory = new TreeSet<>(Comparator.comparing(InstantDetection::getDetectionTime));
   private final InstantDetection firstDetection;
   private Duration historyDuration;
   private final UUID id = UUID.randomUUID();

   /** Represents the pose of the latest instant detection. */
   private final MutableReferenceFrame detectionFrame = new MutableReferenceFrame("persistentDetection_" + id.toString().substring(0, 5),
                                                                                  ReferenceFrame.getWorldFrame());

   private double stabilityConfidenceThreshold;
   private double stabilityDetectionFrequency;

   private boolean isValid = false;
   private final Notification hasBecomeValidNotification = new Notification();
   private boolean readyForDeletion = false;

   public PersistentDetection(InstantDetection firstDetection, double stabilityConfidenceThreshold, double stabilityDetectionFrequency, double historyDuration)
   {
      this.firstDetection = firstDetection;

      addDetection(firstDetection);
      setStabilityConfidenceThreshold(stabilityConfidenceThreshold);
      setStabilityDetectionFrequency(stabilityDetectionFrequency);
      setHistoryDuration(TimeTools.durationOfSeconds(historyDuration));
   }

   /**
    * Add a new instant detection to the history of detections.
    */
   public void addDetection(InstantDetection newDetection)
   {
      detectionHistory.add(newDetection);
      newDetection.setPersistentDetectionID(id);

      detectionFrame.update(transformToWorld -> transformToWorld.set(getMostRecentDetection().getPose()));
   }

   /**
    * Clears history items older than {@code now - historyDuration}, except for the most recent detection.
    * @param now Present time instant.
    */
   public void updateHistory(Instant now)
   {
      // Remove detections that are too old
      detectionHistory.removeIf(detection -> detectionExpired(detection, now) && !detection.equals(getMostRecentDetection()));

      if (!isValid)
      {
         if (isOldEnough(now))
         {
            System.out.println("CONF: " + getAverageConfidence());
            System.out.println("FREQ: " + getDetectionFrequencyDecaying(now));
            if (isStable(now))
            {
               isValid = true;
               hasBecomeValidNotification.set();
            }
            else
            {
               markForDeletion();
            }
         }
      }
   }

   public InstantDetection getOldestDetection()
   {
      return detectionHistory.first();
   }

   /**
    * @return The most recent {@link InstantDetection} added to the history,
    *       based on the detection's {@link java.time.Instant}.
    */
   public InstantDetection getMostRecentDetection()
   {
      return detectionHistory.last();
   }

   public Class<?> getInstantDetectionClass()
   {
      return getMostRecentDetection().getClass();
   }

   public String getDetectedObjectClass()
   {
      return getMostRecentDetection().getDetectedObjectClass();
   }

   public String getDetectedObjectName()
   {
      return getMostRecentDetection().getDetectedObjectName();
   }

   public int getHistorySize()
   {
      return detectionHistory.size();
   }

   /**
    * Set the duration of detection history stored by this object.
    * {@link InstantDetection} objects older than this duration will be removed from the history.
    *
    * @param historyDurationSeconds Number of seconds of history to store. Must be positive.
    */
   public void setHistoryDuration(double historyDurationSeconds)
   {
      setHistoryDuration(TimeTools.durationOfSeconds(historyDurationSeconds));
   }

   /**
    * Set the duration of detection history stored by this object.
    * {@link InstantDetection} objects older than this duration will be removed from the history when {@link #updateHistory} is called.
    *
    * @param historyDuration Duration of history to store. Must be positive.
    */
   public void setHistoryDuration(Duration historyDuration)
   {
      if (historyDuration.isNegative())
         throw new IllegalArgumentException("History duration must be a positive duration");

      this.historyDuration = historyDuration;
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

   public boolean isValid()
   {
      return isValid;
   }

   public Notification hasBecomeValid()
   {
      return hasBecomeValidNotification;
   }

   /**
    * Calculates the average confidence of the {@link InstantDetection}s in the stored history.
    * {@link #updateHistory} should be called before this method to get the up-to-date value!
    * @return Average confidence of {@link InstantDetection}s in history
    */
   public double getAverageConfidence()
   {
      double confidenceSum = 0.0;
      for (InstantDetection detection : detectionHistory)
      {
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

   public double getDetectionFrequencyDecaying(Instant now)
   {
      Instant oldestDetectionTime = detectionHistory.first().getDetectionTime();

      long periodNanos = oldestDetectionTime.until(now, ChronoUnit.NANOS);
      return getHistorySize() / Conversions.nanosecondsToSeconds(periodNanos);
   }

   private boolean detectionExpired(InstantDetection detection, Instant now)
   {
      return detection.getDetectionTime().isBefore(now.minus(historyDuration));
   }

   public SortedSet<InstantDetection> getDetectionHistory()
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

   public void setStabilityConfidenceThreshold(double stabilityConfidenceThreshold)
   {
      this.stabilityConfidenceThreshold = stabilityConfidenceThreshold;
   }

   public void setStabilityDetectionFrequency(double stabilityDetectionFrequency)
   {
      this.stabilityDetectionFrequency = stabilityDetectionFrequency;
   }

   public UUID getID()
   {
      return id;
   }

   public ReferenceFrame getDetectionFrame()
   {
      return detectionFrame.getReferenceFrame();
   }
}

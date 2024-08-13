package us.ihmc.perception.detections;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.Comparator;
import java.util.Iterator;
import java.util.SortedSet;
import java.util.UUID;
import java.util.concurrent.ConcurrentSkipListSet;

public class PersistentDetection
{
   public static final UUID NULL_DETECTION_ID = new UUID(0L, 0L);

   // The first element of this set is the oldest detection, and the last element is the most recent detection
   private final SortedSet<InstantDetection> detectionHistory = new ConcurrentSkipListSet<>(Comparator.comparing(InstantDetection::getDetectionTime));
   private final InstantDetection firstDetection;
   private Duration historyDuration;
   private final UUID id = UUID.randomUUID();

   /** An alpha filtered frame representing the pose of the latest detection. */
   private final MutableReferenceFrame filteredDetectionFrame = new MutableReferenceFrame("persistentDetection_" + id.toString().substring(0, 5),
                                                                                          ReferenceFrame.getWorldFrame());
   private double filterAlpha;

   private double stabilityConfidenceThreshold;
   private double stabilityDetectionFrequency;

   private boolean isValid = false;
   private final Notification hasBecomeValidNotification = new Notification();
   private boolean readyForDeletion = false;

   public PersistentDetection(InstantDetection firstDetection,
                              double filterAlpha,
                              double stabilityConfidenceThreshold,
                              double stabilityDetectionFrequency,
                              double historyDuration)
   {
      this.firstDetection = firstDetection;

      // Initialize filtered frame with first detection pose
      filteredDetectionFrame.update(transformToWorld -> transformToWorld.set(firstDetection.getPose()));

      addDetection(firstDetection);
      setFilterAlpha(filterAlpha);
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

      filteredDetectionFrame.update(transformToWorld -> transformToWorld.interpolate(new RigidBodyTransform(getMostRecentPose()), filterAlpha));
   }

   /**
    * Clears history items older than {@code now - historyDuration}, except for the most recent detection.
    * @param now Present time instant.
    */
   public void updateHistory(Instant now)
   {
      // Remove detections that are too old
      Iterator<InstantDetection> historyIterator = detectionHistory.iterator();
      while (historyIterator.hasNext())
      {
         InstantDetection instantDetection = historyIterator.next();
         if (detectionExpired(instantDetection, now) && !instantDetection.equals(getMostRecentDetection()))
         {
            instantDetection.destroy();
            historyIterator.remove();
         }
      }

      if (!isValid)
      {
         if (isOldEnough(now))
         {
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

   public Pose3DReadOnly getMostRecentPose()
   {
      return getMostRecentDetection().getPose();
   }

   public Point3DReadOnly getMostRecentPosition()
   {
      return getMostRecentDetection().getPose().getPosition();
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

   public void destroy()
   {
      for (InstantDetection instantDetection : detectionHistory)
      {
         instantDetection.destroy();
      }
   }

   public void setFilterAlpha(double filterAlpha)
   {
      this.filterAlpha = filterAlpha;
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

   public ReferenceFrame getFilteredDetectionFrame()
   {
      return filteredDetectionFrame.getReferenceFrame();
   }
}

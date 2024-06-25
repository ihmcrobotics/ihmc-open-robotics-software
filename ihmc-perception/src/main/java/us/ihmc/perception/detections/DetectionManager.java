package us.ihmc.perception.detections;

import org.jetbrains.annotations.NotNull;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.HashSet;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.UUID;

public class DetectionManager
{
   private final Set<PersistentDetection> persistentDetections = new HashSet<>();
   private final Object persistentDetectionsLock = new Object();

   private double matchDistanceSquared;
   private double defaultStabilityThreshold;
   private double defaultStabilityFrequency;
   private double defaultHistorySeconds;

   public DetectionManager()
   {
      this(1.0, 0.5, 5.0, 1.0);
   }

   public DetectionManager(double matchDistanceThreshold, double defaultStabilityThreshold, double defaultStabilityFrequency, double defaultHistorySeconds)
   {
      setMatchDistanceThreshold(matchDistanceThreshold);
      setDefaultStabilityThreshold(defaultStabilityThreshold);
      setDefaultStabilityFrequency(defaultStabilityFrequency);
      setDefaultHistorySeconds(defaultHistorySeconds);
   }

   /**
    * Adds an instant detection of a specified type to the {@code DetectionManager}.
    * The added detection will be matched to a {@link PersistentDetection}, or a new
    * {@link PersistentDetection} will be created if no match is found.
    * This method is thread safe only when called on different detection class types.
    * When detections of the same type are being added, this method is NOT thread safe.
    * @param newInstantDetection The new {@link InstantDetection} to add.
    * @param classType The class of the type of {@link InstantDetection} being added.
    *                  E.g. {@link us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection}
    *                  or {@link us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection}
    * @param <T> Class extending {@link InstantDetection}. Must match the passed in {@code classType}.
    */
   public <T extends InstantDetection> void addDetection(T newInstantDetection, Class<T> classType)
   {
      DetectionPair<T> bestMatch = null;

      Set<PersistentDetection> sameTypePersistentDetections = getDetectionsOfType(classType);
      for (PersistentDetection persistentDetection : sameTypePersistentDetections)
      {
         // matches must be of the same class
         if (persistentDetection.getDetectedObjectClass().equals(newInstantDetection.getDetectedObjectClass()))
         {
            double distanceSquared = persistentDetection.getMostRecentDetection()
                                                        .getPose()
                                                        .getPosition()
                                                        .distanceSquared(newInstantDetection.getPose().getPosition());
            // matches must be close enough
            if (distanceSquared < matchDistanceSquared)
            {
               if (bestMatch == null || bestMatch.getDistanceSquared() > distanceSquared)
                  bestMatch = new DetectionPair<>(persistentDetection, newInstantDetection);
            }
         }
      }

      synchronized (persistentDetectionsLock)
      {
         if (bestMatch != null)
            bestMatch.confirmDetectionMatch();
         else
            persistentDetections.add(new PersistentDetection(newInstantDetection,
                                                             defaultStabilityThreshold,
                                                             defaultStabilityFrequency,
                                                             defaultHistorySeconds));
      }
   }

   /**
    * Adds a set of instant detections of a specified type to the {@code DetectionManager}.
    * The added detections will be matched to existing {@link PersistentDetection}s,
    * or if a new detection is unmatched, a new {@link PersistentDetection} will be created.
    * This method is thread safe only when called on different detection class types.
    * When detections of the same type are being added, this method is NOT thread safe.
    * @param newInstantDetections Set of {@link InstantDetection}s, ideally all from the same detection frame.
    * @param classType The class of the type of {@link InstantDetection} being added.
    *                  E.g. {@link us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection}
    *                  or {@link us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection}
    * @param <T> Class extending {@link InstantDetection}. Must match the passed in {@code classType}.
    */
   public <T extends InstantDetection> void addDetections(Set<T> newInstantDetections, Class<T> classType)
   {
      PriorityQueue<DetectionPair<T>> possibleMatches = new PriorityQueue<>();

      // Keep track of unmatched detections
      Set<T> unmatchedNewDetections = new HashSet<>(newInstantDetections);
      Set<PersistentDetection> unmatchedPersistentDetections = getDetectionsOfType(classType);

      // Find all possible matches
      for (PersistentDetection persistentDetection : unmatchedPersistentDetections)
      {
         for (T newInstantDetection : newInstantDetections)
         {
            // matches must be of the same class
            if (persistentDetection.getDetectedObjectClass().equals(newInstantDetection.getDetectedObjectClass()))
            {
               double distanceSquared = persistentDetection.getMostRecentDetection()
                                                           .getPose()
                                                           .getPosition()
                                                           .distanceSquared(newInstantDetection.getPose().getPosition());
               // matches must be close enough
               if (distanceSquared < matchDistanceSquared)
                  possibleMatches.add(new DetectionPair<>(persistentDetection, newInstantDetection));
            }
         }
      }

      Set<DetectionPair<T>> matchedDetections = new HashSet<>();
      while(!unmatchedNewDetections.isEmpty() && !unmatchedPersistentDetections.isEmpty() && !possibleMatches.isEmpty())
      {
         // Get the best match
         DetectionPair<T> detectionPair = possibleMatches.poll();
         PersistentDetection persistentDetection = detectionPair.persistentDetection;
         T newInstantDetection = detectionPair.instantDetection;

         // If it hasn't been used already, update the persistent detection
         if (unmatchedPersistentDetections.contains(persistentDetection)
             && unmatchedNewDetections.contains(newInstantDetection))
         {
            matchedDetections.add(detectionPair);
            unmatchedPersistentDetections.remove(persistentDetection);
            unmatchedNewDetections.remove(newInstantDetection);
         }
      }

      synchronized(persistentDetectionsLock)
      {
         // add the matched new instant detections to the persistent detections' histories
         matchedDetections.forEach(DetectionPair::confirmDetectionMatch);

         // create new persistent detections from unmatched new detections
         for (T unmatchedNewDetection : unmatchedNewDetections)
            persistentDetections.add(new PersistentDetection(unmatchedNewDetection,
                                                             defaultStabilityThreshold,
                                                             defaultStabilityFrequency,
                                                             defaultHistorySeconds));
      }
   }

   public <T extends InstantDetection> Set<PersistentDetection> updateAndGetDetectionsOfType(Class<T> classType)
   {
      synchronized (persistentDetectionsLock)
      {
         updateDetections();
         return getDetectionsOfType(classType);
      }
   }

   @SuppressWarnings("unchecked")
   public <T extends InstantDetection> Set<PersistentDetection> getDetectionsOfType(Class<T> classType)
   {
      Set<PersistentDetection> typeDetections = new HashSet<>();

      synchronized (persistentDetectionsLock)
      {
         for (PersistentDetection persistentDetection : persistentDetections)
            if (persistentDetection.getInstantDetectionClass().equals(classType))
               typeDetections.add((PersistentDetection) persistentDetection);
      }

      return typeDetections;
   }

   public Set<PersistentDetection> updateAndGetDetections()
   {
      updateDetections();
      return getDetections();
   }

   public Set<PersistentDetection> getDetections()
   {
      synchronized(persistentDetectionsLock)
      {
         return new HashSet<>(persistentDetections);
      }
   }

   public PersistentDetection getDetection(UUID detectionID)
   {
      Set<PersistentDetection> detections = getDetections();
      for (PersistentDetection detection : detections)
      {
         if (detection.getID().equals(detectionID))
            return detection;
      }

      return null;
   }

   public <T extends InstantDetection> PersistentDetection getDetection(UUID detectionID, Class<T> classType)
   {
      Set<PersistentDetection> detections = getDetectionsOfType(classType);
      for (PersistentDetection detection : detections)
      {
         if (detection.getID().equals(detectionID))
            return detection;
      }

      return null;
   }

   public void updateDetections()
   {
      updateDetections(Instant.now());
   }

   public void updateDetections(Instant now)
   {
      synchronized (persistentDetectionsLock)
      {
         Iterator<PersistentDetection> detectionIterator = persistentDetections.iterator();
         while (detectionIterator.hasNext())
         {
            PersistentDetection detection = detectionIterator.next();
            if (detection.isReadyForDeletion())
               detectionIterator.remove();
            else
               detection.updateHistory(now);
         }
      }
   }

   public void setMatchDistanceThreshold(double matchDistance)
   {
      matchDistanceSquared = matchDistance * matchDistance;
   }

   public void setDefaultStabilityThreshold(double defaultStabilityThreshold)
   {
      this.defaultStabilityThreshold = defaultStabilityThreshold;
   }

   public void setDefaultStabilityFrequency(double defaultStabilityFrequency)
   {
      this.defaultStabilityFrequency = defaultStabilityFrequency;
   }

   public void setDefaultHistorySeconds(double historyLengthSeconds)
   {
      this.defaultHistorySeconds = historyLengthSeconds;
   }

   public void setDefaultHistoryDuration(Duration historyDuration)
   {
      this.defaultHistorySeconds = TimeTools.toDoubleSeconds(historyDuration);
   }

   private record DetectionPair<T extends InstantDetection>(PersistentDetection persistentDetection, T instantDetection)
         implements Comparable<DetectionPair<? extends InstantDetection>>
   {
      public double getDistanceSquared()
      {
         return persistentDetection.getMostRecentDetection().getPose().getPosition().distanceSquared(instantDetection.getPose().getPosition());
      }

      public void confirmDetectionMatch()
      {
         persistentDetection.addDetection(instantDetection);
      }

      @Override
      public int compareTo(@NotNull DetectionPair<? extends InstantDetection> other)
      {
         return Double.compare(getDistanceSquared(), other.getDistanceSquared());
      }
   }
}

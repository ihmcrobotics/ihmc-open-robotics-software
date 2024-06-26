package us.ihmc.perception.detections;

import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.UUID;

public class DetectionManager
{
   /** All persistent detections managed by this class */
   private final Set<PersistentDetection> persistentDetections = new HashSet<>();
   /** Set of detections that have become valid for the first time. Only accessed by the SceneGraph*/
   private final Set<PersistentDetection> newlyValidDetections = new HashSet<>();
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
    * Adds a detection using {@link #addDetections}
    */
   public <T extends InstantDetection> void addDetection(T newInstantDetection)
   {
      addDetections(List.of(newInstantDetection));
   }

   /**
    * Adds a set of instant detections of a specified type to the {@code DetectionManager}.
    * The added detections will be matched to existing {@link PersistentDetection}s,
    * or if a new detection is unmatched, a new {@link PersistentDetection} will be created.
    * This method is thread safe only when called on different detection class types.
    * When detections of the same type are being added, this method is NOT thread safe.
    *
    * @param newInstantDetections Set of {@link InstantDetection}s, ideally all from the same detection frame.
    */
   public <T extends InstantDetection> void addDetections(List<T> newInstantDetections)
   {
      List<PersistentDetection> persistentDetectionsOfClass = getDetectionsOfType(newInstantDetections.get(0).getClass());

      // Find all possible matches
      PriorityQueue<DetectionPair> possibleMatches = new PriorityQueue<>();
      for (PersistentDetection persistentDetection : persistentDetectionsOfClass)
      {
         for (InstantDetection newInstantDetection : newInstantDetections)
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
                  possibleMatches.add(new DetectionPair(persistentDetection, newInstantDetection));
            }
         }
      }

      // Keep track of unmatched detections
      Set<InstantDetection> unmatchedNewDetections = new HashSet<>(newInstantDetections);
      Set<DetectionPair> matchedDetections = new HashSet<>();
      while (!unmatchedNewDetections.isEmpty() && !persistentDetectionsOfClass.isEmpty() && !possibleMatches.isEmpty())
      {
         // Get the best match
         DetectionPair detectionPair = possibleMatches.poll();
         PersistentDetection persistentDetection = detectionPair.getPersistentDetection();
         InstantDetection newInstantDetection = detectionPair.getInstantDetection();

         // If it hasn't been used already, update the persistent detection
         if (persistentDetectionsOfClass.contains(persistentDetection)
             && unmatchedNewDetections.contains(newInstantDetection))
         {
            matchedDetections.add(detectionPair);
            persistentDetectionsOfClass.remove(persistentDetection);
            unmatchedNewDetections.remove(newInstantDetection);
         }
      }

      synchronized(persistentDetectionsLock)
      {
         // add the matched new instant detections to the persistent detections' histories
         matchedDetections.forEach(DetectionPair::confirmDetectionMatch);

         // create new persistent detections from unmatched new detections
         for (InstantDetection unmatchedNewDetection : unmatchedNewDetections)
            persistentDetections.add(new PersistentDetection(unmatchedNewDetection,
                                                             defaultStabilityThreshold,
                                                             defaultStabilityFrequency,
                                                             defaultHistorySeconds));
      }
   }

   public <T extends InstantDetection> List<PersistentDetection> updateAndGetDetectionsOfType(Class<T> classType)
   {
      synchronized (persistentDetectionsLock)
      {
         updateDetections();
         return getDetectionsOfType(classType);
      }
   }

   public List<PersistentDetection> getDetectionsOfType(Class<?> classType)
   {
      List<PersistentDetection> typeDetections = new LinkedList<>();

      synchronized (persistentDetectionsLock)
      {
         for (PersistentDetection persistentDetection : persistentDetections)
            if (persistentDetection.getInstantDetectionClass().equals(classType))
               typeDetections.add(persistentDetection);
      }

      return typeDetections;
   }

   public Set<PersistentDetection> getDetections()
   {
      synchronized(persistentDetectionsLock)
      {
         return new HashSet<>(persistentDetections);
      }
   }

   public Set<PersistentDetection> getNewlyValidDetections()
   {
      return newlyValidDetections;
   }

   public void clearNewlyValidDetections()
   {
      newlyValidDetections.clear();
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
            {
               detection.updateHistory(now);
               if (detection.hasBecomeValid().poll())
                  newlyValidDetections.add(detection);
            }
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
}

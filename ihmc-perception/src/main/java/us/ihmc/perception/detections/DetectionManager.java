package us.ihmc.perception.detections;

import org.jetbrains.annotations.NotNull;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

public class DetectionManager
{
   private final Set<PersistentDetection<? extends InstantDetection>> persistentDetections = new HashSet<>();
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

   public <T extends InstantDetection> void addDetections(Set<T> newInstantDetections, Class<T> classType)
   {
      PriorityQueue<DetectionPair<T>> possibleMatches = new PriorityQueue<>();

      // Keep track of unmatched detections
      Set<T> unmatchedNewDetections = new HashSet<>(newInstantDetections);
      Set<PersistentDetection<T>> unmatchedPersistentDetections = getDetectionsOfType(classType);

      // Find all possible matches
      for (PersistentDetection<T> persistentDetection : unmatchedPersistentDetections)
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
         PersistentDetection<T> persistentDetection = detectionPair.persistentDetection;
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
            persistentDetections.add(new PersistentDetection<>(unmatchedNewDetection,
                                                               defaultStabilityThreshold,
                                                               defaultStabilityFrequency,
                                                               defaultHistorySeconds));
      }
   }

   public <T extends InstantDetection> Set<PersistentDetection<T>> updateAndGetDetectionsOfType(Class<T> classType)
   {
      synchronized (persistentDetectionsLock)
      {
         updateDetections();
         return getDetectionsOfType(classType);
      }
   }

   public <T extends InstantDetection> Set<PersistentDetection<T>> getDetectionsOfType(Class<T> classType)
   {
      Set<PersistentDetection<T>> typeDetections = new HashSet<>();

      synchronized (persistentDetectionsLock)
      {
         for (PersistentDetection<? extends InstantDetection> persistentDetection : persistentDetections)
            if (persistentDetection.getInstantDetectionClass().equals(classType))
               typeDetections.add((PersistentDetection<T>) persistentDetection);
      }

      return typeDetections;
   }

   public Set<PersistentDetection<? extends InstantDetection>> getDetections()
   {
      synchronized(persistentDetectionsLock)
      {
         return new HashSet<>(persistentDetections);
      }
   }

   public void updateDetections()
   {
      updateDetections(Instant.now());
   }

   public void updateDetections(Instant now)
   {
      synchronized (persistentDetectionsLock)
      {
         persistentDetections.forEach(detection -> detection.updateHistory(now));
      }
   }

   public boolean removeDetection(PersistentDetection<? extends InstantDetection> detectionToRemove)
   {
      return persistentDetections.remove(detectionToRemove);
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

   private record DetectionPair<T extends InstantDetection>(PersistentDetection<T> persistentDetection, T instantDetection)
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

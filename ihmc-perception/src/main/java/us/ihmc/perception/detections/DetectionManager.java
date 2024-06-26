package us.ihmc.perception.detections;

import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.UUID;

public class DetectionManager
{
   private final Set<PersistentDetection> persistentDetections = new HashSet<>();
   private final Object persistentDetectionsLock = new Object();

   private double maxMatchDistanceSquared;
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
    * For example, you can add YOLO  & CenterPose detection concurrently,
    * but you cannot add two sets of YOLO detections concurrently.
    * When detections of the same type are being added, this method is NOT thread safe.
    *
    * @param newInstantDetections Set of {@link InstantDetection}s, ideally all from the same detection frame.
    */
   public <T extends InstantDetection> void addDetections(List<T> newInstantDetections)
   {
      List<PersistentDetection> persistentDetectionsOfClass = getDetectionsOfType(newInstantDetections.get(0).getClass());

      // Find all possible matches, sorting by distance to get the closest matches
      PriorityQueue<DetectionPair> potentialMatches = new PriorityQueue<>();
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
               if (distanceSquared < maxMatchDistanceSquared)
                  potentialMatches.add(new DetectionPair(persistentDetection, newInstantDetection));
            }
         }
      }

      // Build a new set of the best-aligning potential matches one by one, ensuring no duplicates
      Set<InstantDetection> remainingNewDetections = new HashSet<>(newInstantDetections);
      List<PersistentDetection> remainingPersistentDetections = persistentDetectionsOfClass;
      List<DetectionPair> validAndBestMatches = new ArrayList<>();
      while (!remainingNewDetections.isEmpty() && !remainingPersistentDetections.isEmpty() && !potentialMatches.isEmpty())
      {
         // Get the next closest match
         DetectionPair detectionPair = potentialMatches.poll();
         PersistentDetection persistentDetection = detectionPair.getPersistentDetection();
         InstantDetection newInstantDetection = detectionPair.getInstantDetection();

         // If it hasn't been used already, validate the match
         if (remainingPersistentDetections.contains(persistentDetection)
             && remainingNewDetections.contains(newInstantDetection))
         {
            validAndBestMatches.add(detectionPair);
            remainingPersistentDetections.remove(persistentDetection);
            remainingNewDetections.remove(newInstantDetection);
         }
      }

      synchronized(persistentDetectionsLock)
      {
         // add the matched new instant detections to the persistent detections' histories
         for (DetectionPair match : validAndBestMatches)
            match.getPersistentDetection().addDetection(match.getInstantDetection());

         // create new persistent detections from unmatched new detections
         for (InstantDetection unmatchedNewDetection : remainingNewDetections)
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
      List<PersistentDetection> typeDetections = new ArrayList<>();

      synchronized (persistentDetectionsLock)
      {
         for (PersistentDetection persistentDetection : persistentDetections)
            if (persistentDetection.getInstantDetectionClass().equals(classType))
               typeDetections.add(persistentDetection);
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
      for (PersistentDetection detection : getDetectionsOfType(classType))
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
      maxMatchDistanceSquared = matchDistance * matchDistance;
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

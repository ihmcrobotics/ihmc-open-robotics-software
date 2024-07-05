package us.ihmc.perception.detections;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.robotics.time.TimeTools;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

public class DetectionManager
{
   /** All persistent detections managed by this class */
   private final Set<PersistentDetection> persistentDetections = new HashSet<>();
   /** Set of detections that have become valid for the first time. Only accessed by the SceneGraph*/
   private final Set<PersistentDetection> newlyValidDetections = new HashSet<>();
   private final Object persistentDetectionsLock = new Object();

   private final DetectionManagerSettings settings = new DetectionManagerSettings();
   private final ROS2StoredPropertySet<DetectionManagerSettings> settingsSync;

   public DetectionManager(ROS2PublishSubscribeAPI ros2)
   {
      settingsSync = ros2 == null ? null : new ROS2StoredPropertySet<>(ros2, PerceptionAPI.DETECTION_MANAGER_SETTINGS, settings);
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
               if (distanceSquared < settings.getMaxMatchDistanceSquared())
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
                                                             settings.getPoseFilterAlpha(),
                                                             settings.getAcceptanceAverageConfidence(),
                                                             settings.getStabilityDetectionFrequency(),
                                                             settings.getDetectionHistoryDuration()));
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
      if (settingsSync != null)
         settingsSync.updateAndPublishThrottledStatus();

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
               {
                  detection.setStabilityConfidenceThreshold(settings.getStabilityAverageConfidence());
                  newlyValidDetections.add(detection);
               }
            }
         }
      }
   }

   public void setPoseFilterAlpha(double poseFilterAlpha)
   {
      settings.setPoseFilterAlpha(poseFilterAlpha);
   }

   public void setMatchDistanceThreshold(double matchDistance)
   {
      settings.setMaxMatchDistanceSquared(matchDistance * matchDistance);
   }

   public void setAcceptanceAverageConfidence(double acceptanceAverageConfidence)
   {
      settings.setAcceptanceAverageConfidence(acceptanceAverageConfidence);
   }

   public void setStabilityAverageConfidence(double stabilityAverageConfidence)
   {
      settings.setStabilityAverageConfidence(stabilityAverageConfidence);
   }

   public void setStabilityDetectionFrequency(double stabilityDetectionFrequency)
   {
      settings.setStabilityDetectionFrequency(stabilityDetectionFrequency);
   }

   public void setDetectionHistoryDuration(double historyLengthSeconds)
   {
      settings.setDetectionHistoryDuration(historyLengthSeconds);
   }

   public void setDetectionHistoryDuration(Duration historyDuration)
   {
      settings.setDetectionHistoryDuration(TimeTools.toDoubleSeconds(historyDuration));
   }
}

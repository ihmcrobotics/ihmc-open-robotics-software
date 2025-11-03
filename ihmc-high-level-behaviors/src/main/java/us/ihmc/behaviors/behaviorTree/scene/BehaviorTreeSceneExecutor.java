package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.DetectionPair;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.LongSupplier;

public class BehaviorTreeSceneExecutor extends BehaviorTreeSceneState
{
   private final YOLOv8DetectionExecutor yolo;
   private final IsaacROSFoundationPoseCommunicatorMap foundationPose;

   private final List<BehaviorTreeSceneObjectExecutor> objects;
   private final ConcurrentLinkedQueue<List<InstantDetection>> detectionQueue = new ConcurrentLinkedQueue<>();

   // Persistent detection management
   private final Set<PersistentDetection> persistentDetections = new HashSet<>();
   private static final double MAX_MATCH_DISTANCE_SQUARED = 0.25; // 0.5m threshold
   private static final double POSE_FILTER_ALPHA = 0.5;
   private static final double ACCEPTANCE_CONFIDENCE = 0.5;
   private static final double STABILITY_FREQUENCY = 1.0;
   private static final double HISTORY_DURATION = 2.0;

   public BehaviorTreeSceneExecutor(CRDTInfo crdtInfo,
                                    LongSupplier idSupplier,
                                    ROS2SyncedRobotModel syncedRobot,
                                    YOLOv8DetectionExecutor yolo,
                                    IsaacROSFoundationPoseCommunicatorMap foundationPose)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.yolo = yolo;
      this.foundationPose = foundationPose;

      objects = (List) super.objects;

      yolo.addDetectionConsumerCallback(detectionQueue::add);
   }

   public void update()
   {
      // Process detections from the queue
      List<InstantDetection> detections;
      while ((detections = detectionQueue.poll()) != null)
      {
         addDetections(detections);
      }

      // Update persistent detections
      updatePersistentDetections(Instant.now());

      for (BehaviorTreeSceneObjectExecutor object : objects)
      {
         object.update();
      }
   }

   private void addDetections(List<InstantDetection> newInstantDetections)
   {
      List<PersistentDetection> persistentDetectionsOfClass = getDetectionsOfType(newInstantDetections.get(0).getClass());

      // Find all possible matches, sorting by distance to get the closest matches
      PriorityQueue<DetectionPair> potentialMatches = new PriorityQueue<>();
      for (PersistentDetection persistentDetection : persistentDetectionsOfClass)
      {
         for (InstantDetection newInstantDetection : newInstantDetections)
         {
            // Matches must be of the same class
            if (persistentDetection.getDetectedObjectClass().equals(newInstantDetection.getDetectedObjectClass()))
            {
               DetectionPair pair = new DetectionPair(persistentDetection, newInstantDetection);
               // Matches must be close enough
               if (pair.getDistanceSquared() < MAX_MATCH_DISTANCE_SQUARED)
                  potentialMatches.add(pair);
            }
         }
      }

      // Build a new set of the best-aligning potential matches one by one, ensuring no duplicates
      Set<InstantDetection> remainingNewDetections = new HashSet<>(newInstantDetections);
      List<PersistentDetection> remainingPersistentDetections = new ArrayList<>(persistentDetectionsOfClass);
      List<DetectionPair> validAndBestMatches = new ArrayList<>();
      while (!remainingNewDetections.isEmpty() && !remainingPersistentDetections.isEmpty() && !potentialMatches.isEmpty())
      {
         // Get the next closest match
         DetectionPair detectionPair = potentialMatches.poll();
         PersistentDetection persistentDetection = detectionPair.getPersistentDetection();
         InstantDetection newInstantDetection = detectionPair.getInstantDetection();

         // If it hasn't been used already, validate the match
         if (remainingPersistentDetections.contains(persistentDetection) && remainingNewDetections.contains(newInstantDetection))
         {
            validAndBestMatches.add(detectionPair);
            remainingPersistentDetections.remove(persistentDetection);
            remainingNewDetections.remove(newInstantDetection);
         }
      }

      // Add the matched new instant detections to the persistent detections' histories
      for (DetectionPair match : validAndBestMatches)
         match.getPersistentDetection().addDetection(match.getInstantDetection());

      // Create new persistent detections from unmatched new detections
      for (InstantDetection unmatchedNewDetection : remainingNewDetections)
         persistentDetections.add(new PersistentDetection(unmatchedNewDetection,
                                                          POSE_FILTER_ALPHA,
                                                          ACCEPTANCE_CONFIDENCE,
                                                          STABILITY_FREQUENCY,
                                                          HISTORY_DURATION));
   }

   private void updatePersistentDetections(Instant now)
   {
      Iterator<PersistentDetection> detectionIterator = persistentDetections.iterator();
      while (detectionIterator.hasNext())
      {
         PersistentDetection detection = detectionIterator.next();
         if (detection.isDestroyed())
         {
            detectionIterator.remove();
         }
         else if (detection.isReadyForDeletion())
         {
            detection.destroy();
            detectionIterator.remove();
         }
         else
         {
            detection.updateHistory(now);
            if (detection.hasBecomeValid().poll())
            {
               // Detection has become valid - could trigger scene object creation here
            }
         }
      }
   }

   private List<PersistentDetection> getDetectionsOfType(Class<?> classType)
   {
      List<PersistentDetection> typeDetections = new ArrayList<>();
      for (PersistentDetection persistentDetection : persistentDetections)
         if (persistentDetection.getInstantDetectionClass().equals(classType))
            typeDetections.add(persistentDetection);
      return typeDetections;
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(BehaviorTreeSceneObjectStateMessage message)
   {
      return new BehaviorTreeSceneObjectExecutor(message.getId(), crdtInfo, message.getTypeAsString());
   }
}

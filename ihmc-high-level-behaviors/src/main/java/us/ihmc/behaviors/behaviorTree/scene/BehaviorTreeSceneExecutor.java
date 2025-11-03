package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;

import java.time.Instant;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.LongSupplier;

public class BehaviorTreeSceneExecutor extends BehaviorTreeSceneState
{
   private final YOLOv8DetectionExecutor yolo;
   private final IsaacROSFoundationPoseCommunicatorMap foundationPose;

   private final List<BehaviorTreeSceneObjectExecutor> objects;

   private final ConcurrentLinkedQueue<List<InstantDetection>> instantDetectionQueue = new ConcurrentLinkedQueue<>();
   private final Set<PersistentDetection> matchedThisTick = new HashSet<>();
   private final Set<PersistentDetection> persistentDetections = new HashSet<>();

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

      yolo.addDetectionConsumerCallback(instantDetectionQueue::add);

   }

   public void update()
   {
      while (!instantDetectionQueue.isEmpty())
         triageInstantDetections(instantDetectionQueue.poll());

      updatePersistentDetections(Instant.now());

      for (BehaviorTreeSceneObjectExecutor object : objects)
      {
         object.update();
      }
   }

   private void triageInstantDetections(List<InstantDetection> newInstantDetections)
   {
      matchedThisTick.clear();

      // Greedy single-pass matching: for each new detection, find closest unmatched persistent detection
      for (InstantDetection newDetection : newInstantDetections)
      {
         PersistentDetection bestMatch = null;
         double bestDistanceSquared = 0.25; // max 0.5m radius

         for (PersistentDetection candidate : persistentDetections)
         {
            if (matchedThisTick.contains(candidate))
               continue;

            // Check detection type (e.g., YOLO, FoundationPose, etc)
            if (!candidate.getInstantDetectionClass().equals(newDetection.getClass()))
               continue;

            // Check object class (e.g., "bottle")
            if (!candidate.getDetectedObjectClass().equals(newDetection.getDetectedObjectClass()))
               continue;

            double distanceSquared = candidate.getMostRecentDetection().getPose().getPosition().distanceSquared(newDetection.getPose().getPosition());

            // Track closest match within threshold
            if (distanceSquared < bestDistanceSquared)
            {
               bestDistanceSquared = distanceSquared;
               bestMatch = candidate;
            }
         }

         // Update existing persistent detection or create new one
         if (bestMatch != null)
         {
            matchedThisTick.add(bestMatch);
            bestMatch.addDetection(newDetection);
         }
         else
         {
            double poseFilterAlpha = 0.5;
            double acceptanceConfidence = 0.5;
            double stabilityFrequency = 1.0;
            double historyDuration = 2.0;
            persistentDetections.add(new PersistentDetection(newDetection, poseFilterAlpha, acceptanceConfidence, stabilityFrequency, historyDuration));
         }
      }
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

   @Override
   protected BehaviorTreeSceneObjectState buildObject(BehaviorTreeSceneObjectStateMessage message)
   {
      return new BehaviorTreeSceneObjectExecutor(message.getId(), crdtInfo, message.getTypeAsString());
   }
}

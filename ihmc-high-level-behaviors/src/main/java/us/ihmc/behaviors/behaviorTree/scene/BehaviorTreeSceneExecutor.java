package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashSet;
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
   private final List<PersistentDetection> persistentDetections = new ArrayList<>();
   private final List<PersistentDetection> oldUnstableDetections = new ArrayList<>();
   private final transient FramePose3D detectionPose = new FramePose3D();
   private final transient RigidBodyTransform detectionTransform = new RigidBodyTransform();

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

      updatePersistentDetections();

      for (BehaviorTreeSceneObjectExecutor object : objects)
      {
         object.update(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
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
            double acceptanceConfidence = 0.25;
            double stabilityFrequency = 1.0;
            double historyDuration = 2.0;
            persistentDetections.add(new PersistentDetection(newDetection, poseFilterAlpha, acceptanceConfidence, stabilityFrequency, historyDuration));
         }
      }
   }

   private void updatePersistentDetections()
   {
      Instant now = Instant.now();
      oldUnstableDetections.clear();
      for (PersistentDetection detection : persistentDetections)
      {
         detection.updateHistory(now);

         if (detection.isReadyForDeletion() || (detection.isOldEnough(now) && detection.getHistorySize() < 2))
            oldUnstableDetections.add(detection);
      }

      for (PersistentDetection persistentDetectionToRemove : oldUnstableDetections)
      {
         persistentDetectionToRemove.destroy();
         persistentDetections.remove(persistentDetectionToRemove);
      }

      persistentDetections.sort((a, b) -> Integer.compare(b.getHistorySize(), a.getHistorySize()));
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(long id, CRDTInfo crdtInfo, IsaacROSFoundationPoseObject objectType)
   {
      return new BehaviorTreeSceneObjectExecutor(id, crdtInfo, objectType);
   }

   @Override
   public void toMessage(BehaviorTreeSceneStateMessage message)
   {
      super.toMessage(message);

      Instant now = Instant.now();
      message.getPersistentDetections().clear();
      for (PersistentDetection persistentDetection : persistentDetections)
      {
         PersistentDetectionStatusMessage status = message.getPersistentDetections().add();
         String last4Digits = String.format("%04d", Math.abs(persistentDetection.getID().getLeastSignificantBits() % 10000));
         status.setId(last4Digits);
         status.setObjectClass(persistentDetection.getDetectedObjectClass());
         status.setDecayingFrequency(persistentDetection.getDetectionFrequencyDecaying(now));
         status.setHistorySize(persistentDetection.getHistorySize());
         status.setIsStable(persistentDetection.isStable(now));

         detectionPose.setToZero(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
         detectionPose.set(persistentDetection.getFilteredTransformToCamera());
         detectionPose.changeFrame(ReferenceFrame.getWorldFrame());
         detectionTransform.set(detectionPose);
         MessageTools.toMessage(detectionTransform, status.getTransformToWorld());
      }
   }

   public List<PersistentDetection> getPersistentDetections()
   {
      return persistentDetections;
   }
}

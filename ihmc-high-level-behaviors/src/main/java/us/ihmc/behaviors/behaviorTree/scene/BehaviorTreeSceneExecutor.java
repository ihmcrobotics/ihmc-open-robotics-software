package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.sensors.ImageSensor;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.LongSupplier;

public class BehaviorTreeSceneExecutor extends BehaviorTreeSceneState
{
   private final ImageSensor imageSensor;
   private final YOLOv8DetectionExecutor yolo;
   private final IsaacROSFoundationPoseCommunicatorMap foundationPose;
   private final TerrainMapData terrainMapData;

   private final List<BehaviorTreeSceneObjectExecutor> objects;

   private final ConcurrentLinkedQueue<List<InstantDetection>> instantDetectionQueue = new ConcurrentLinkedQueue<>();
   private final Set<PersistentDetection> matchedThisTick = new HashSet<>();
   private final List<PersistentDetection> persistentDetections = new ArrayList<>();
   private final List<PersistentDetection> oldUnstableDetections = new ArrayList<>();
   private final PersistentDetectionMessageTool persistentDetectionMessageTool = new PersistentDetectionMessageTool();

   public BehaviorTreeSceneExecutor(CRDTInfo crdtInfo,
                                    LongSupplier idSupplier,
                                    ROS2SyncedRobotModel syncedRobot,
                                    ImageSensor imageSensor,
                                    YOLOv8DetectionExecutor yolo,
                                    IsaacROSFoundationPoseCommunicatorMap foundationPose,
                                    TerrainMapData terrainMapData)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.imageSensor = imageSensor;
      this.yolo = yolo;
      this.foundationPose = foundationPose;
      this.terrainMapData = terrainMapData;

      objects = (List) super.objects;

      if (yolo != null)
         yolo.addDetectionConsumerCallback(instantDetectionQueue::add);

      if (foundationPose != null)
      {
         IsaacROSFoundationPoseCommunicator mustardCommunicator = foundationPose.get(IsaacROSFoundationPoseObject.MUSTARD);
         mustardCommunicator.enable(true);
         mustardCommunicator.addResultCallback(detection -> instantDetectionQueue.add(List.of(detection)));
      }
   }

   public void update()
   {
      while (!instantDetectionQueue.isEmpty())
         triageInstantDetections(instantDetectionQueue.poll());

      updatePersistentDetections();

      for (BehaviorTreeSceneObjectExecutor object : objects)
         object.update();
   }

   @Override
   public void setSyncedRobot(ROS2SyncedRobotModel syncedRobot)
   {
      super.setSyncedRobot(syncedRobot);

      for (BehaviorTreeSceneObjectExecutor object : objects)
         object.setSyncedRobot(syncedRobot);
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
            persistentDetections.add(new PersistentDetection(newDetection,
                                                             poseFilterAlpha.getValue(),
                                                             acceptanceConfidence.getValue(),
                                                             stabilityFrequency.getValue(),
                                                             historyDuration.getValue()));
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
   protected BehaviorTreeSceneObjectState buildObject(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      return switch (BehaviorTreeSceneObjectType.values()[definition.getObjectType()])
      {
         case YOLO_ONLY, FOUNDATION_POSE -> new BehaviorTreeSceneObjectExecutor(id, crdtInfo, syncedRobot, definition);
         case COMPOSITE_FRAME -> new BehaviorTreeSceneCompositeFrameExecutor(id, crdtInfo, syncedRobot, definition, this);
         case DOOR_PANEL -> new BehaviorTreeSceneDoorPanelExecutor(id, crdtInfo, syncedRobot, definition);
         case DOOR_FRAME -> new BehaviorTreeSceneDoorFrameExecutor(id, crdtInfo, syncedRobot, definition, this);
         case APPROACH_TABLE -> new BehaviorTreeSceneApproachTableExecutor(id, crdtInfo, syncedRobot, definition, this);
      };
   }

   @Override
   public void toMessage(BehaviorTreeSceneStateMessage message)
   {
      super.toMessage(message);

      Instant now = Instant.now();
      message.getPersistentDetections().clear();
      for (PersistentDetection persistentDetection : persistentDetections)
         persistentDetectionMessageTool.toMessage(syncedRobot, now, persistentDetection, message.getPersistentDetections().add());
   }

   public List<PersistentDetection> getPersistentDetections()
   {
      return persistentDetections;
   }

   public ImageSensor getImageSensor()
   {
      return imageSensor;
   }

   public YOLOv8DetectionExecutor getYOLO()
   {
      return yolo;
   }

   public TerrainMapData getTerrainMap()
   {
      return terrainMapData;
   }
}

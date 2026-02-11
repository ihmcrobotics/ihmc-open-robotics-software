package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.LongSupplier;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.CAMERA_TO_OPTICAL_ROTATION;

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

   private final RawImagePublisher imagePublisher;
   private final ROS2MutableFrame annotatedImageFrame;

   public BehaviorTreeSceneExecutor(ROS2Node ros2Node,
                                    CRDTInfo crdtInfo,
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
      {
         yolo.addDetectionConsumerCallback(instantDetectionQueue::add);
      }

      if (foundationPose != null)
      {
         IsaacROSFoundationPoseCommunicator mustardCommunicator = foundationPose.get(IsaacROSFoundationPoseObject.MUSTARD);
         mustardCommunicator.enable(true);
         mustardCommunicator.addResultCallback(detection -> instantDetectionQueue.add(List.of(detection)));
      }

      imagePublisher = new RawImagePublisher(ros2Node);
      annotatedImageFrame = new ROS2MutableFrame("VLMAnnotatedImageFrame", ReferenceFrame.getWorldFrame());
   }

   public void update()
   {
      while (!instantDetectionQueue.isEmpty())
         triageInstantDetections(instantDetectionQueue.poll());

      updatePersistentDetections();

      for (BehaviorTreeSceneObjectExecutor object : objects)
         object.update();
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
   protected BehaviorTreeSceneObjectState buildObject(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      return switch (BehaviorTreeSceneObjectType.values()[definition.getObjectType()])
      {
         case DOOR_PANEL -> new BehaviorTreeSceneDoorPanelExecutor(id, crdtInfo, syncedRobot, definition);
         case DOOR_FRAME -> new BehaviorTreeSceneDoorFrameExecutor(id, crdtInfo, syncedRobot, definition, this);
         default -> new BehaviorTreeSceneObjectExecutor(id, crdtInfo, syncedRobot, definition);
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

   public void publishYOLOAnnotatedImage()
   {
      List<YOLOv8InstantDetection> yoloDetections = new ArrayList<>();
      Map<YOLOv8InstantDetection, Integer> detectionIdMap = new HashMap<>();
      RawImage colorImage = null;
      RawImage annotatedImage = null;

      for (PersistentDetection persistentDetection : persistentDetections)
      {
         if (persistentDetection.getInstantDetectionClass() != YOLOv8InstantDetection.class || !persistentDetection.isStable())
            continue;

         YOLOv8InstantDetection detection = (YOLOv8InstantDetection) persistentDetection.getMostRecentDetection();
         if (colorImage == null)
         {
            colorImage = detection.getColorImage().get();
            if (colorImage == null)
               continue;

            annotatedImage = new RawImage(colorImage);
         }
         yoloDetections.add(detection);
         detectionIdMap.put(detection, persistentDetection.getID());
      }

      if (annotatedImage == null)
         return;

      YOLOv8Tools.drawObjectOutlines(colorImage.getCpuImageMat(), annotatedImage.getCpuImageMat(), yoloDetections, detection ->
      {
         int id = detectionIdMap.get(detection);
         return id + ": " + detection.getDetectedObjectName();
      });

      RigidBodyTransform transformToWorld = new RigidBodyTransform(annotatedImage.getTransformToWorld());
      transformToWorld.appendOrientation(CAMERA_TO_OPTICAL_ROTATION);
      annotatedImageFrame.setNewTransformToParent(transformToWorld);
      annotatedImageFrame.update();

      imagePublisher.publishImage(PerceptionAPI.YOLO_VLM_ANNOTATED_IMAGE, annotatedImage, annotatedImageFrame);
      imagePublisher.publishImage(PerceptionAPI.YOLO_VML_ANNOTATED_IMAGE_CAMERA_INFO, annotatedImage, annotatedImageFrame);

      colorImage.release();
      annotatedImage.release();
   }

   public List<PersistentDetection> getPersistentDetections()
   {
      return persistentDetections;
   }

   public ImageSensor getImageSensor()
   {
      return imageSensor;
   }

   public TerrainMapData getTerrainMap()
   {
      return terrainMapData;
   }
}

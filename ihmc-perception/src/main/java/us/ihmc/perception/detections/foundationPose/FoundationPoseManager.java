package us.ihmc.perception.detections.foundationPose;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;

import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

public class FoundationPoseManager
{
   private static final Duration YOLO_DETECTION_TIMEOUT = Duration.ofSeconds(1);
   private static final Duration GOOD_DETECTION_DURATION = Duration.ofSeconds(2);

   private static final int DELTA = 10;
   private static final AtomicLong ID = new AtomicLong(0L);

   private final Map<PersistentDetection, Instant> allYOLODetections;
   private final Map<String, PersistentDetection> trackedYOLODetections;

   private final ROS2FoundationPoseCommunicator communicator;

   private final RepeatingTaskThread updateThread;

   private final Set<String> objectsToTrack;

   public FoundationPoseManager(ROS2Node ros2Node, ImageSensor imageSensor, int colorKey, int depthKey)
   {
      allYOLODetections = new HashMap<>();
      trackedYOLODetections = new HashMap<>();

      communicator = new ROS2FoundationPoseCommunicator(ros2Node, imageSensor, colorKey, depthKey);

      objectsToTrack = new HashSet<>();

      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Update", this::update);
      updateThread.setFrequencyLimit(30.0).startRepeating();
   }

   /**
    * Specify whether an objects of a class should be tracked.
    * <p>
    * If {@code true}, the manager will send requests to the FoundationPose process to track the specified objects when seen by YOLO.
    * If {@code false}, the manager will send remove messages for the objects that are being tracked,
    * and will not send requests for the object class.
    *
    * @param objectClass The class of object (should match the mesh file name used by FoundationPose)
    * @param shouldTrack Whether objects of the class should be tracked.
    */
   public void setObjectClassTracking(String objectClass, boolean shouldTrack)
   {
      String objectClassNoNumber = objectClass.replaceAll("\\d+$", "");
      LogTools.info(objectClassNoNumber);
      String fullObjectClassName = objectClassNoNumber + ".obj";
      if (!FoundationPoseTools.getAvailableMeshes().contains(fullObjectClassName))
      {
         LogTools.error("FoundationPose cannot track objects of class " + objectClass + " because we don't have a mesh for it");
      }
      else
      {
         if (shouldTrack)
            objectsToTrack.add(fullObjectClassName);
         else
            objectsToTrack.remove(fullObjectClassName);
      }
   }

   /**
    * Set whether FoundationPose should be tracking objects.
    * <p>
    * If {@code true}, this object will send requests to the FoundationPose process to track all objects seen by YOLO.
    * If {@code false}, this object will send remove messages for all objects that are being tracked,
    * and will not send any requests.
    *
    * @param active Whether FoundationPose should be tracking objects.
    */
   public void setActive(boolean active)
   {
      if (active)
         objectsToTrack.addAll(FoundationPoseTools.getAvailableMeshes());
      else
         objectsToTrack.clear();
   }

   private void onNewDetection(PersistentDetection detection)
   {
      if (YOLOv8InstantDetection.class.equals(detection.getInstantDetectionClass()))
      {
         synchronized (allYOLODetections)
         {
            allYOLODetections.put(detection, detection.getMostRecentDetection().getDetectionTime());
         }
      }
   }

   private void onDetectionRemoved(PersistentDetection detection)
   {
      if (YOLOv8InstantDetection.class.equals(detection.getInstantDetectionClass()))
      {
         synchronized (allYOLODetections)
         {
            allYOLODetections.remove(detection);
            trackedYOLODetections.values().remove(detection);
         }
      }
      else if (FoundationPoseInstantDetection.class.equals(detection.getInstantDetectionClass()))
      {
         String objectId = detection.getDetectedObjectName();
         communicator.remove(objectId);
         trackedYOLODetections.remove(objectId);
      }
   }

   /**
    * Add a callback that's ran when a new result is received from FoundationPose
    *
    * @param resultCallback Callback to run. The FoundationPose result will be provided to it.
    */
   public void addResultCallback(Consumer<List<InstantDetection>> resultCallback)
   {
      communicator.addResultCallback(result ->
                                     {
                                        FoundationPoseInstantDetection instantDetection = new FoundationPoseInstantDetection(result.getMeshFileAsString(),
                                                                                                                             result.getObjectIdAsString(),
                                                                                                                             result.getObjectPose().getPose(),
                                                                                                                             MessageTools.toInstant(result.getTimestamp()));
                                        resultCallback.accept(List.of(instantDetection));
                                     });
   }

   public void destroy()
   {
      communicator.destroy();
      updateThread.blockingKill();
   }

   private void update()
   {
      Instant now = Instant.now();
      Instant yoloTimeout = now.minus(YOLO_DETECTION_TIMEOUT);
      Instant goodDetectionTime = now.minus(GOOD_DETECTION_DURATION);

      synchronized (allYOLODetections)
      {
         // Update the last time each YOLO detection was "bad"
         allYOLODetections.forEach((yoloDetection, lastBadDetectionTime) ->
                                   {
                                      boolean detectionIsBad = !yoloDetection.isStable(now) ||
                                                               yoloDetection.getMostRecentDetection().getDetectionTime().isBefore(yoloTimeout) ||
                                                               boundingBoxIsAtEdge((YOLOv8InstantDetection) yoloDetection.getMostRecentDetection());
                                      if (detectionIsBad)
                                         allYOLODetections.put(yoloDetection, now);
                                   });

         // Look through tracked detections and stop tracking ones that are bad
         Iterator<Entry<String, PersistentDetection>> detectionIterator = trackedYOLODetections.entrySet().iterator();
         while (detectionIterator.hasNext())
         {
            Entry<String, PersistentDetection> entry = detectionIterator.next();
            String objectId = entry.getKey();
            PersistentDetection detection = entry.getValue();

            // If detection was bad within the last second, we shouldn't be tracking it
            Instant lastBadDetectionTime = allYOLODetections.get(detection);
            if ((!detection.getDetectedObjectName().contains("charge") && (lastBadDetectionTime == null || lastBadDetectionTime.isAfter(goodDetectionTime))) ||
                !objectTrackingIsActive(detection.getDetectedObjectName()))
            {
               LogTools.debug("Removing {}", objectId);
               communicator.remove(objectId);
               detectionIterator.remove();
            }
         }
      }

      // Look through untracked detections, and determine which we want to track
      Set<PersistentDetection> untrackedYOLODetections;
      synchronized (allYOLODetections)
      {
         untrackedYOLODetections = new HashSet<>(allYOLODetections.keySet());
         untrackedYOLODetections.removeAll(trackedYOLODetections.values());
      }
      Set<PersistentDetection> detectionsToTrack = new HashSet<>();
      for (PersistentDetection detection : untrackedYOLODetections)
      {
         // Track detections that haven't been bad for at least a second
         Instant lastBadDetectionTime = allYOLODetections.get(detection);
         if (lastBadDetectionTime != null && lastBadDetectionTime.isBefore(goodDetectionTime) && objectTrackingIsActive(detection.getDetectedObjectName()))
            detectionsToTrack.add(detection);
      }

      // Send requests to track the detections we want to track
      for (PersistentDetection detection : detectionsToTrack)
      {
         YOLOv8InstantDetection yoloDetection = (YOLOv8InstantDetection) detection.getMostRecentDetection();
         String objectId = detection.getDetectedObjectName() + "_fp_#" + ID.getAndIncrement();
         String meshFile = FoundationPoseTools.getYOLOClassToObjectMeshMap().get(detection.getDetectedObjectName());
         LogTools.debug("Requesting tracking for {}", objectId);
         communicator.track(objectId, meshFile, yoloDetection.getObjectMask(), yoloDetection.getColorImage(), yoloDetection.getDepthImage());
         synchronized (allYOLODetections)
         {
            trackedYOLODetections.put(objectId, detection);
         }
      }
   }

   private boolean boundingBoxIsAtEdge(YOLOv8InstantDetection detection)
   {
      RawImage colorImage = detection.getColorImage().get();
      BoundingBox2DReadOnly boundingBox = detection.getBoundingBox();

      // Ensure image exists
      if (colorImage == null)
         return false;

      int imageWidth = colorImage.getWidth();
      int imageHeight = colorImage.getHeight();
      colorImage.release();

      int minX = (int) Math.round(boundingBox.getMinX());
      int minY = (int) Math.round(boundingBox.getMinY());
      int maxX = (int) Math.round(boundingBox.getMaxX());
      int maxY = (int) Math.round(boundingBox.getMaxY());

      return minX <= DELTA || minY <= DELTA || maxX >= imageWidth - 1 - DELTA || maxY >= imageHeight - 1 - DELTA;
   }

   private boolean objectTrackingIsActive(String yoloClass)
   {
      String meshName = FoundationPoseTools.getYOLOClassToObjectMeshMap().get(yoloClass);
      if (meshName == null)
         return false;

      String objectClass = meshName.split("\\.")[0] + ".obj";
      return objectsToTrack.contains(objectClass);
   }
}
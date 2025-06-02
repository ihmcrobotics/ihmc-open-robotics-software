package us.ihmc.perception.detections.foundationPose;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;

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
   private static final int DELTA = 10;
   private static final AtomicLong ID = new AtomicLong(0L);

   private final Set<PersistentDetection> allYOLODetections;
   private final Map<String, PersistentDetection> trackedYOLODetections;

   private final ROS2FoundationPoseCommunicator communicator;

   private final RepeatingTaskThread updateThread;

   public FoundationPoseManager(ROS2Node ros2Node, ImageSensor imageSensor, int colorKey, int depthKey)
   {
      allYOLODetections = new HashSet<>();
      trackedYOLODetections = new HashMap<>();

      communicator = new ROS2FoundationPoseCommunicator(ros2Node, imageSensor, colorKey, depthKey);

      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Update", this::update);
      updateThread.setFrequencyLimit(10.0).startRepeating();
   }

   /**
    * Register callbacks between this class and the {@link DetectionManager}.
    *
    * @param detectionManager The {@link DetectionManager}.
    */
   public void registerDetectionManagerCallbacks(DetectionManager detectionManager)
   {
      addResultCallback(detectionManager::addDetections);
      detectionManager.addNewlyValidDetectionCallback(this::onNewDetection);
      detectionManager.addDetectionRemovedCallback(this::onDetectionRemoved);
   }

   private void onNewDetection(PersistentDetection detection)
   {
      if (YOLOv8InstantDetection.class.equals(detection.getInstantDetectionClass()))
      {
         synchronized (allYOLODetections)
         {
            allYOLODetections.add(detection);
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
         communicator.remove(detection.getDetectedObjectName());
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
                                                                                              result.getObjectPose(),
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
      Instant secondAgo = now.minusSeconds(1);

      // Look through Tracked detections and determine which we should stop tracking
      synchronized (allYOLODetections)
      {
         Iterator<Entry<String, PersistentDetection>> detectionIterator = trackedYOLODetections.entrySet().iterator();
         while (detectionIterator.hasNext())
         {
            Entry<String, PersistentDetection> entry = detectionIterator.next();
            String objectId = entry.getKey();
            PersistentDetection detection = entry.getValue();

            // If detection is not stable or if it's been more than a second since last detection, we stop tracking the object
            if (!detection.isStable(now) || detection.getMostRecentDetection().getDetectionTime().isBefore(secondAgo))
            {
               communicator.remove(objectId);
               detectionIterator.remove();
            }
         }
      }

      // Look through untracked detections, and determine which we want to track
      Set<PersistentDetection> untrackedYOLODetections;
      synchronized (allYOLODetections)
      {
         untrackedYOLODetections = new HashSet<>(allYOLODetections);
         untrackedYOLODetections.removeAll(trackedYOLODetections.values());
      }
      Set<PersistentDetection> detectionsToTrack = new HashSet<>();
      for (PersistentDetection detection : untrackedYOLODetections)
      {
         YOLOv8InstantDetection yoloDetection = (YOLOv8InstantDetection) detection.getMostRecentDetection();
         RawImage colorImage = yoloDetection.getColorImage().get();
         BoundingBox2DReadOnly boundingBox = yoloDetection.getBoundingBox();

         // Ensure image exists
         if (colorImage == null)
            continue;

         // If the detection is stable and the bounding box is not at the edge, we want to track the detection
         if (detection.isStable(now) && boundingBoxIsNotAtEdge(boundingBox, colorImage.getWidth(), colorImage.getHeight()))
            detectionsToTrack.add(detection);

         colorImage.release();
      }

      // Send requests to track the detections we want to track
      for (PersistentDetection detection : detectionsToTrack)
      {
         YOLOv8InstantDetection yoloDetection = (YOLOv8InstantDetection) detection.getMostRecentDetection();
         String objectId = detection.getDetectedObjectName() + "_fp_#" + ID.getAndIncrement();
         String meshFile = FoundationPoseTools.getYOLOClassToObjectMeshMap().get(detection.getDetectedObjectName());
         communicator.track(objectId, meshFile, yoloDetection.getObjectMask(), yoloDetection.getColorImage(), yoloDetection.getDepthImage());
         synchronized (allYOLODetections)
         {
            trackedYOLODetections.put(objectId, detection);
         }
      }
   }

   private boolean boundingBoxIsNotAtEdge(BoundingBox2DReadOnly boundingBox, int imageWidth, int imageHeight)
   {
      int minX = (int) Math.round(boundingBox.getMinX());
      int minY = (int) Math.round(boundingBox.getMinY());
      int maxX = (int) Math.round(boundingBox.getMaxX());
      int maxY = (int) Math.round(boundingBox.getMaxY());

      return minX > DELTA && minY > DELTA && maxX < imageWidth - 1 - DELTA && maxY < imageHeight - 1 - DELTA;
   }
}

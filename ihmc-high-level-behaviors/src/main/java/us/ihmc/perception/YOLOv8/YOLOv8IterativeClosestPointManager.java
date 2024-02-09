package us.ihmc.perception.YOLOv8;

import org.ddogleg.struct.Tuple3;
import perception_msgs.msg.dds.YOLOv8ParametersMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.RestartableThread;

import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class YOLOv8IterativeClosestPointManager
{
   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final WorkspaceResourceDirectory pointCloudDirectory = new WorkspaceResourceDirectory(YOLOv8DetectableObject.class, "/yoloICPPointClouds/");

   private final ROS2Helper ros2Helper;

   private final YOLOv8ObjectDetector yoloDetector = new YOLOv8ObjectDetector();
   private RawImage yoloDetectionImage;
   private RawImage icpEnvironmentDepthImage;
   private long lastYoloImageSequenceNumber = -1;
   private long lastICPImageSequenceNumber = -1;
   private final Set<YOLOv8Detection> oldDetections = ConcurrentHashMap.newKeySet();
   private final RestartableThread yoloICPThread;
   private final Lock imageUpdateLock = new ReentrantLock();
   private final Condition newImagesAvailable = imageUpdateLock.newCondition();
   private final Notification readyToRunNotification = new Notification();

   private final ConcurrentHashMap<YOLOv8Detection, YOLOv8IterativeClosestPointNodeCombo> detectionToWorkerMap = new ConcurrentHashMap<>();

   private final IHMCROS2Input<YOLOv8ParametersMessage> yoloParameterSubscription;
   private float yoloConfidenceThreshold = 0.3f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;

   public YOLOv8IterativeClosestPointManager(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
      yoloICPThread = new RestartableThread("YOLODetector", this::runYOLODetection);
      readyToRunNotification.set();

      yoloParameterSubscription = ros2Helper.subscribe(PerceptionAPI.YOLO_PARAMETERS);
      yoloParameterSubscription.addCallback(parametersMessage ->
      {
         yoloConfidenceThreshold = parametersMessage.getConfidenceThreshold();
         yoloNMSThreshold = parametersMessage.getNonMaximumSuppressionThreshold();
         yoloSegmentationThreshold = parametersMessage.getSegmentationThreshold();
      });
   }

   public void setDetectionImages(RawImage yoloColorImage, RawImage icpDepthImage)
   {
      yoloColorImage.get();
      icpDepthImage.get();
      imageUpdateLock.lock();
      try
      {
         if (yoloDetectionImage != null)
            yoloDetectionImage.release();
         if (icpEnvironmentDepthImage != null)
            icpEnvironmentDepthImage.release();

         yoloDetectionImage = yoloColorImage;
         icpEnvironmentDepthImage = icpDepthImage;
         newImagesAvailable.signal();
      }
      finally
      {
         imageUpdateLock.unlock();
      }
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         for (Map.Entry<YOLOv8Detection, YOLOv8IterativeClosestPointNodeCombo> detectionComboEntry : detectionToWorkerMap.entrySet())
         {
            if (!detectionComboEntry.getValue().updateSceneGraph(sceneGraph, modificationQueue, detectionComboEntry.getKey()))
               detectionToWorkerMap.remove(detectionComboEntry.getKey());
         }
      });
   }

   public void start()
   {
      yoloICPThread.start();
   }

   public void destroy()
   {
      yoloICPThread.stop();
      segmenter.destroy();
      extractor.destroy();
      openCLManager.destroy();

      yoloDetector.destroy();

      if (yoloDetectionImage != null)
         yoloDetectionImage.release();
      if (icpEnvironmentDepthImage != null)
         icpEnvironmentDepthImage.release();
   }

   private void runYOLODetection()
   {
      RawImage yoloColorImageCopy = null;
      RawImage icpDepthImageCopy = null;

      imageUpdateLock.lock();
      try
      {
         // Wait for new images to arrive
         while ((yoloDetectionImage == null || yoloDetectionImage.isEmpty() || yoloDetectionImage.getSequenceNumber() == lastYoloImageSequenceNumber)
                && (icpEnvironmentDepthImage == null || icpEnvironmentDepthImage.isEmpty()
                    || icpEnvironmentDepthImage.getSequenceNumber() == lastICPImageSequenceNumber))
         {
            newImagesAvailable.await();
         }

         // Copy the images
         yoloColorImageCopy = yoloDetectionImage.get();
         icpDepthImageCopy = icpEnvironmentDepthImage.get();

         lastYoloImageSequenceNumber = yoloDetectionImage.getSequenceNumber();
         lastICPImageSequenceNumber = icpEnvironmentDepthImage.getSequenceNumber();

      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         imageUpdateLock.unlock();
      }

      // Run YOLO and ICP in a non-blocking way
      if (yoloColorImageCopy != null && icpDepthImageCopy != null && readyToRunNotification.poll())
      {
         runYOLOAndICP(yoloColorImageCopy, icpDepthImageCopy);
         yoloColorImageCopy.release();
         icpDepthImageCopy.release();
      }
   }

   private void runYOLOAndICP(RawImage yoloColorImage, RawImage icpDepthImage)
   {
      // Get the results and object masks
      YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(yoloColorImage, yoloConfidenceThreshold, yoloNMSThreshold);
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getICPSegmentationImages(yoloSegmentationThreshold);

      // Get the new detections
      Set<YOLOv8Detection> newDetections = yoloResults.getICPDetections();

      /* Organize the detections.
       * Old and new detections which match will be returned in the map, and removed from their respective sets.
       * The new detections remaining in the set are considered to be new instances of the detected object.
       * The old detections remaining in the set are considered to be old instances of the object no longer in the scene.
       */
      Map<YOLOv8Detection, YOLOv8Detection> oldNewMatchingDetections = organizeDetections(detectionToWorkerMap.values(), oldDetections, newDetections);

      // Remove the old detection (objects no longer in the scene) and associated stuff
      oldDetections.removeIf(oldDetection -> detectionToWorkerMap.get(oldDetection).destroy());

      // Run ICP on the matching detections
      for (Map.Entry<YOLOv8Detection, YOLOv8Detection> oldNewPair : oldNewMatchingDetections.entrySet())
      {
         YOLOv8Detection oldDetection = oldNewPair.getKey();
         YOLOv8Detection newDetection = oldNewPair.getValue();

         if (objectMasks.get(newDetection) != null)
         {
            detectionToWorkerMap.get(oldDetection).runICP(newDetection, icpDepthImage, objectMasks.get(newDetection));

            // replace old key with new one
            detectionToWorkerMap.put(newDetection, detectionToWorkerMap.remove(oldDetection));
            // replace old detection with new detection (new one is old now)
            oldDetections.remove(oldDetection);
            oldDetections.add(newDetection);
         }
      }

      // Create new ICP workers for brand-new detections
      for (YOLOv8Detection newDetection : newDetections)
      {
         detectionToWorkerMap.put(newDetection, new YOLOv8IterativeClosestPointNodeCombo(ros2Helper, pointCloudDirectory, openCLManager));
         oldDetections.add(newDetection);
      }

      for (RawImage mask : objectMasks.values())
         mask.release();
      yoloResults.destroy();

      readyToRunNotification.set();
   }

   /**
    * Old and new detections which match will be returned in the map, and removed from their respective sets.
    * The new detections remaining in the set are considered to be new instances of the detected object.
    * The old detections remaining in the set are considered to be old instances of the object no longer in the scene.
    *
    * @param oldDetectionCombos Previous YOLOv8 detections. MODIFIED: old detections which were matched to new ones will be removed from the set.
    * @param newDetections New YOLOv8 detections. MODIFIED: new detections which were matched to new ones will be removed from the set.
    * @return Map of closest (matching) old and new detections
    */
   private Map<YOLOv8Detection, YOLOv8Detection> organizeDetections(Collection<YOLOv8IterativeClosestPointNodeCombo> oldDetectionCombos,
                                                                    Set<YOLOv8Detection> oldDetections,
                                                                    Set<YOLOv8Detection> newDetections)
   {
      // Priority queue of all possible detection pairs ordered by distance between them
      PriorityQueue<Tuple3<YOLOv8Detection, YOLOv8Detection, Double>> matchingDetections = new PriorityQueue<>(Comparator.comparingDouble(Tuple3::getD2));

      // Add all detections which possibly match to the queue
      for (YOLOv8IterativeClosestPointNodeCombo oldDetectionCombo : oldDetectionCombos)
      {
         YOLOv8Detection oldDetection = oldDetectionCombo.getLastDetection();
         if (oldDetection != null)
         {
            for (YOLOv8Detection newDetection : newDetections)
            {
               double distanceSquared = MathTools.square(oldDetection.x() - newDetection.x()) + MathTools.square(oldDetection.y() - newDetection.y());

               if (oldDetection.objectClass() == newDetection.objectClass() && distanceSquared < oldDetectionCombo.getDistanceThreshold())
                  matchingDetections.add(new Tuple3<>(oldDetection, newDetection, distanceSquared));
            }
         }
      }

      // Match up old and new detections
      Map<YOLOv8Detection, YOLOv8Detection> oldNewMatchingDetectionMap = new HashMap<>();
      while (!oldDetections.isEmpty() && !newDetections.isEmpty())
      {
         Tuple3<YOLOv8Detection, YOLOv8Detection, Double> bestMatchDetections = matchingDetections.poll();
         if (bestMatchDetections == null)
            break;

         YOLOv8Detection oldMatchDetection = bestMatchDetections.getD0();
         YOLOv8Detection newMatchDetection = bestMatchDetections.getD1();
         if (oldDetections.contains(oldMatchDetection) && newDetections.contains(newMatchDetection))
         {
            oldDetections.remove(oldMatchDetection);
            newDetections.remove(newMatchDetection);
            oldNewMatchingDetectionMap.put(oldMatchDetection, newMatchDetection);
         }
      }

      return oldNewMatchingDetectionMap;
   }
}

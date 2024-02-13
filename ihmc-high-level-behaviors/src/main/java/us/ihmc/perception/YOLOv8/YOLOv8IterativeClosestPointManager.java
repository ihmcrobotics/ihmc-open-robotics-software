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

import java.util.Comparator;
import java.util.HashSet;
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
   private final RestartableThread yoloICPThread;
   private final Lock imageUpdateLock = new ReentrantLock();
   private final Condition newImagesAvailable = imageUpdateLock.newCondition();
   private final Notification readyToRunNotification = new Notification();

   private final Map<YOLOv8IterativeClosestPointNodeCombo, YOLOv8IterativeClosestPointNodeCombo> yoloICPNodeComboSet = new ConcurrentHashMap<>();

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
         for (YOLOv8IterativeClosestPointNodeCombo yoloICPCombo : yoloICPNodeComboSet.keySet())
         {
            if (!yoloICPCombo.updateSceneGraph(sceneGraph, modificationQueue))
               yoloICPNodeComboSet.remove(yoloICPCombo);
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
      yoloColorImage.get();
      icpDepthImage.get();

      // Get the results and object masks
      YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(yoloColorImage, yoloConfidenceThreshold, yoloNMSThreshold);
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getICPSegmentationImages(yoloSegmentationThreshold);

      // Get the new detections
      Set<YOLOv8Detection> newDetections = yoloResults.getICPDetections();

      // Give the YOLO ICP combos new detections
      Set<YOLOv8IterativeClosestPointNodeCombo> unmatchedYOLOICPCombos = organizeDetections(yoloICPNodeComboSet, newDetections);

      // Signal combos which did not receive a new detection to consider destruction
      unmatchedYOLOICPCombos.forEach(YOLOv8IterativeClosestPointNodeCombo::destroy);

      // Create new combos for new detections
      for (YOLOv8Detection newDetection : newDetections)
      {
         YOLOv8IterativeClosestPointNodeCombo newCombo = new YOLOv8IterativeClosestPointNodeCombo(ros2Helper, pointCloudDirectory, openCLManager);
         newCombo.setDetection(newDetection);
         yoloICPNodeComboSet.put(newCombo, newCombo);
      }

      // Run ICP on all new detections
      for (YOLOv8IterativeClosestPointNodeCombo yoloICPCombo : yoloICPNodeComboSet.keySet())
      {
         RawImage mask = objectMasks.get(yoloICPCombo.getLastDetection());
         if (mask != null)
            yoloICPCombo.runICP(icpDepthImage, mask);
      }

      icpDepthImage.release();
      yoloColorImage.release();

      for (RawImage mask : objectMasks.values())
         mask.release();
      yoloResults.destroy();

      readyToRunNotification.set();
   }

   /**
    * YOLO ICP combos will receive the best match new detection. Matched new detections will be removed from their set.
    * YOLO ICP combos which did not receive a new detection will be returned in a set.
    * The new detections remaining in the set are considered to be new instances of the detected object.
    * The returned YOLO ICP combos are considered to be instances of objects no longer in the scene.
    *
    * @param detectionCombos Existing YOLO ICP combos. UNMODIFIED.
    * @param newDetections New YOLO detections, not yet matched to the YOLO ICP combos.
    *                      MODIFIED: detections which were matched to YOLO ICP combos will be removed from the set
    * @return Set of YOLO ICP combos which did not receive a new detection
    */
   private Set<YOLOv8IterativeClosestPointNodeCombo> organizeDetections(Map<YOLOv8IterativeClosestPointNodeCombo, YOLOv8IterativeClosestPointNodeCombo> detectionCombos,
                                                                        Set<YOLOv8Detection> newDetections)
   {
      // Priority queue of all possible detection pairs ordered by distance between them
      PriorityQueue<Tuple3<YOLOv8IterativeClosestPointNodeCombo, YOLOv8Detection, Double>> possiblyMatchingDetections
            = new PriorityQueue<>(Comparator.comparingDouble(Tuple3::getD2));

      // Add all detections which possibly match to the queue
      for (YOLOv8IterativeClosestPointNodeCombo oldDetectionCombo : detectionCombos.keySet())
      {
         YOLOv8Detection oldDetection = oldDetectionCombo.getLastDetection();
         if (oldDetection != null)
         {
            for (YOLOv8Detection newDetection : newDetections)
            {
               double distanceSquared = MathTools.square(oldDetection.x() - newDetection.x()) + MathTools.square(oldDetection.y() - newDetection.y());

               if (oldDetection.objectClass() == newDetection.objectClass() && distanceSquared < oldDetectionCombo.getDistanceThreshold())
                  possiblyMatchingDetections.add(new Tuple3<>(oldDetectionCombo, newDetection, distanceSquared));
            }
         }
      }

      // Match up old and new detections, removing each match from the list of unmatched detections
      Set<YOLOv8IterativeClosestPointNodeCombo> unmatchedOldDetections = new HashSet<>(detectionCombos.keySet());
      while (!newDetections.isEmpty() && ! unmatchedOldDetections.isEmpty())
      {
         Tuple3<YOLOv8IterativeClosestPointNodeCombo, YOLOv8Detection, Double> bestMatchDetections = possiblyMatchingDetections.poll();
         if (bestMatchDetections == null)
            break;

         YOLOv8IterativeClosestPointNodeCombo oldDetection = detectionCombos.get(bestMatchDetections.getD0());
         if (unmatchedOldDetections.contains(bestMatchDetections.getD0()) && newDetections.contains(bestMatchDetections.getD1()))
         {
            oldDetection.setDetection(bestMatchDetections.getD1());
            unmatchedOldDetections.remove(bestMatchDetections.getD0());
            newDetections.remove(bestMatchDetections.getD1());
         }
      }

      return unmatchedOldDetections;
   }
}

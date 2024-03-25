package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.tools.thread.RestartableThrottledThread;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class YOLOv8DetectionManager
{
   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final YOLOv8ObjectDetector yoloDetector = new YOLOv8ObjectDetector();
   private RawImage colorImage = null;
   private long lastColorImageSequenceNumber = -1L;
   private RawImage depthImage = null;
   private long lastDepthImageSequenceNumber = -1L;

   private final RestartableThrottledThread yoloDetectionThread;
   private final Lock newImageLock = new ReentrantLock();
   private final Condition newImageAvailable = newImageLock.newCondition();
   private final Notification readyToRunNotification = new Notification();
   private final FrequencyCalculator yoloFrequencyCalculator = new FrequencyCalculator();

   private final Map<YOLOv8DetectionClass, YOLOv8Node> detectedNodes = new ConcurrentHashMap<>();
   private final Map<YOLOv8DetectionClass, YOLOv8SegmentedDetection> detectedObjects = new ConcurrentHashMap<>();
   private final Map<YOLOv8DetectionClass, YOLOv8SegmentedDetection> candidateDetections = new ConcurrentHashMap<>();

   private float yoloConfidenceThreshold = 0.5f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;
   private float candidateAcceptanceThreshold = 0.6f;

   private ReferenceFrame robotFrame = null;

   private boolean destroyed = false;

   public YOLOv8DetectionManager(ROS2Helper ros2Helper)
   {
      yoloDetectionThread = new RestartableThrottledThread("YOLODetector", 5.0, this::runYOLODetection);
      yoloDetectionThread.start();

      readyToRunNotification.set();

      ros2Helper.subscribe(PerceptionAPI.YOLO_PARAMETERS).addCallback(parametersMessage ->
      {
         yoloConfidenceThreshold = parametersMessage.getConfidenceThreshold();
         yoloNMSThreshold = parametersMessage.getNonMaximumSuppressionThreshold();
         yoloSegmentationThreshold = parametersMessage.getSegmentationThreshold();
         candidateAcceptanceThreshold = parametersMessage.getCandidateAcceptanceThreshold();
      });
   }

   public void setDetectionImages(RawImage newColorImage, RawImage newDepthImage)
   {
      newColorImage.get();
      newDepthImage.get();
      newImageLock.lock();
      try
      {
         if (colorImage != null)
            colorImage.release();
         if (depthImage != null)
            depthImage.release();

         colorImage = newColorImage;
         depthImage = newDepthImage;
         newImageAvailable.signal();
      }
      finally
      {
         newImageLock.unlock();
      }
   }

   public void setRobotFrame(ReferenceFrame robotFrame)
   {
      this.robotFrame = robotFrame;
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      destroyed = true;
      yoloDetectionThread.stop();
      segmenter.destroy();
      extractor.destroy();
      openCLManager.destroy();

      yoloDetector.destroy();
      if (colorImage != null)
         colorImage.release();
      if (depthImage != null)
         depthImage.release();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private void runYOLODetection()
   {
      RawImage colorImageCopy = null;
      RawImage depthImageCopy = null;

      newImageLock.lock();
      try
      {
         while (colorImage == null || colorImage.isEmpty() || colorImage.getSequenceNumber() == lastColorImageSequenceNumber
                || depthImage == null || depthImage.isEmpty() || depthImage.getSequenceNumber() == lastDepthImageSequenceNumber)
         {
            newImageAvailable.await();
         }

         // copy the images
         if (colorImage.isAvailable())
            colorImageCopy = colorImage.get();
         if (depthImage.isAvailable())
            depthImageCopy = depthImage.get();

         // update sequence numbers
         lastColorImageSequenceNumber = colorImage.getSequenceNumber();
         lastDepthImageSequenceNumber = depthImage.getSequenceNumber();
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         newImageLock.unlock();
      }

      if (colorImageCopy != null && depthImageCopy != null && robotFrame != null && !destroyed)
      {
         if (readyToRunNotification.poll())
            runYOLOAndSegmentation(colorImageCopy, depthImageCopy, robotFrame);

         colorImageCopy.release();
         depthImageCopy.release();
      }
   }

   private void runYOLOAndSegmentation(RawImage colorImage, RawImage depthImage, ReferenceFrame robotFrame)
   {
      colorImage.get();
      depthImage.get();

      yoloFrequencyCalculator.ping();

      YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(colorImage, yoloConfidenceThreshold, yoloNMSThreshold);

      // Extract stuff from the results
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getSegmentationImages(yoloSegmentationThreshold);
      Set<YOLOv8Detection> newDetections = yoloResults.getDetections();

      // match new detections to existing detections
      Point3D robotPoint = new Point3D(robotFrame.getTransformToWorldFrame().getTranslation());
      for (YOLOv8SegmentedDetection oldDetection : detectedObjects.values())
      {
         double closestDistance = Double.POSITIVE_INFINITY;
         YOLOv8SegmentedDetection bestMatchDetection = null;
         YOLOv8Node yoloNode = detectedNodes.get(oldDetection.getDetection().objectClass());

         Iterator<YOLOv8Detection> newDetectionIterator = newDetections.iterator();
         while (newDetectionIterator.hasNext())
         {
            YOLOv8Detection newDetection = newDetectionIterator.next();

            // Match detections of same object class
            if (newDetection.objectClass() == oldDetection.getDetection().objectClass())
            {
               RawImage mask = objectMasks.get(newDetection);

               int erosionKernelRadius = yoloNode.getMaskErosionKernelRadius();
               double outlierRejectionThreshold = yoloNode.getOutlierFilterThreshold();

               YOLOv8SegmentedDetection segmentedNewDetection = new YOLOv8SegmentedDetection(newDetection,
                                                                                             oldDetection.getDetectionFilter(),
                                                                                             mask,
                                                                                             depthImage,
                                                                                             erosionKernelRadius,
                                                                                             segmenter::removeBackground,
                                                                                             extractor::extractPointCloud,
                                                                                             outlierRejectionThreshold);

               // Best match = detection closest to robot
               double distance = robotPoint.distanceSquared(segmentedNewDetection.getCentroid());
               if (distance < closestDistance)
               {
                  closestDistance = distance;
                  bestMatchDetection = segmentedNewDetection;
               }

               // Remove all new detections of the same object class as existing detections
               newDetectionIterator.remove();
            }
         }

         DetectionFilter filter = oldDetection.getDetectionFilter();
         // If a matching detection was found, register the detection & replace old detection with new one
         if (bestMatchDetection != null)
         {
            detectedObjects.replace(bestMatchDetection.getDetection().objectClass(), bestMatchDetection);
            filter.registerDetection();
         }
         filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
         filter.update();
         // Update detection status of node
         yoloNode.setCurrentlyDetected(bestMatchDetection != null);
      }

      // Same process as above, but for candidate detections
      for (YOLOv8SegmentedDetection oldCandidate : candidateDetections.values())
      {
         double closestDistance = Double.POSITIVE_INFINITY;
         YOLOv8SegmentedDetection bestMatchDetection = null;

         Iterator<YOLOv8Detection> newDetectionIterator = newDetections.iterator();
         while (newDetectionIterator.hasNext())
         {
            YOLOv8Detection newDetection = newDetectionIterator.next();

            if (newDetection.objectClass() == oldCandidate.getDetection().objectClass())
            {
               RawImage mask = objectMasks.get(newDetection);

               YOLOv8SegmentedDetection segmentedNewDetection = new YOLOv8SegmentedDetection(newDetection,
                                                                                             oldCandidate.getDetectionFilter(),
                                                                                             mask,
                                                                                             depthImage,
                                                                                             2,
                                                                                             segmenter::removeBackground,
                                                                                             extractor::extractPointCloud,
                                                                                             2.0);

               double distance = robotPoint.distanceSquared(segmentedNewDetection.getCentroid());
               if (distance < closestDistance)
               {
                  closestDistance = distance;
                  bestMatchDetection = segmentedNewDetection;
               }

               newDetectionIterator.remove();
            }
         }

         DetectionFilter filter = oldCandidate.getDetectionFilter();
         if (bestMatchDetection != null)
         {
            candidateDetections.replace(bestMatchDetection.getDetection().objectClass(), bestMatchDetection);
            filter.registerDetection();
         }

         filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
         filter.update();
      }

      // Only newly detected object classes left; add to candidate detections
      for (YOLOv8Detection newDetection : newDetections)
      {
         DetectionFilter filter = new DetectionFilter((int) yoloFrequencyCalculator.getFrequency(), candidateAcceptanceThreshold);
         filter.registerDetection();
         filter.update();

         YOLOv8SegmentedDetection segmentedNewDetection = new YOLOv8SegmentedDetection(newDetection,
                                                                                       filter,
                                                                                       objectMasks.get(newDetection),
                                                                                       depthImage,
                                                                                       2,
                                                                                       segmenter::removeBackground,
                                                                                       extractor::extractPointCloud,
                                                                                       2.0);

         candidateDetections.put(newDetection.objectClass(), segmentedNewDetection);
      }

      colorImage.release();
      depthImage.release();

      for (RawImage mask : objectMasks.values())
         mask.release();
      yoloResults.destroy();

      readyToRunNotification.set();
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         // Handle candidate detections (add or forget)
         Iterator<Entry<YOLOv8DetectionClass, YOLOv8SegmentedDetection>> candidateIterator = candidateDetections.entrySet().iterator();
         while (candidateIterator.hasNext())
         {
            YOLOv8SegmentedDetection candidateDetection = candidateIterator.next().getValue();
            DetectionFilter filter = candidateDetection.getDetectionFilter();

            if (filter.hasEnoughSamples())
            {
               if (filter.isStableDetectionResult()) // filter has enough samples and detection is stable; add the candidate to scene graph
               {
                  long nodeID = sceneGraph.getNextID().getAndIncrement();
                  YOLOv8Node newYoloNode = new YOLOv8Node(nodeID,
                                                          "YOLO " + candidateDetection.getDetection().objectClass().toString(),
                                                          candidateDetection.getDetection().objectClass(),
                                                          candidateDetection.getObjectPointCloud(),
                                                          candidateDetection.getCentroid());
                  modificationQueue.accept(new SceneGraphNodeAddition(newYoloNode, sceneGraph.getRootNode()));
                  detectedNodes.put(candidateDetection.getDetection().objectClass(), newYoloNode);
                  detectedObjects.put(candidateDetection.getDetection().objectClass(), candidateDetection);
                  candidateDetection.getDetectionFilter().setAcceptanceThreshold(0.2f);
               }

               candidateIterator.remove();
            }
         }
      });

      // Handle existing detections
      Iterator<Entry<YOLOv8DetectionClass, YOLOv8Node>> yoloNodeIterator = detectedNodes.entrySet().iterator();
      while (yoloNodeIterator.hasNext())
      {
         YOLOv8Node yoloNode = yoloNodeIterator.next().getValue();
         YOLOv8SegmentedDetection detection = detectedObjects.get(yoloNode.getDetectionClass());

         // Node may have been removed by user
         if (!sceneGraph.getIDToNodeMap().containsKey(yoloNode.getID()))
         {
            yoloNodeIterator.remove();
            detectedObjects.remove(detection.getDetection().objectClass());
         }
         else // Node still exists; update
         {
            DetectionFilter filter = detection.getDetectionFilter();
            filter.setAcceptanceThreshold(yoloNode.getDetectionAcceptanceThreshold());

            yoloNode.setObjectPointCloud(detection.getObjectPointCloud());
            yoloNode.setObjectCentroid(detection.getCentroid());
            yoloNode.update();
         }
      }
   }
}

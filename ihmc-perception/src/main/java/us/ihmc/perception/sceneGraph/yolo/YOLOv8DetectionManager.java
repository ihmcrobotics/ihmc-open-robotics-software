package us.ihmc.perception.sceneGraph.yolo;

import perception_msgs.msg.dds.YOLOv8ParametersMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Input;
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
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.tools.PerceptionDebugTools;
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

   private final Map<YOLOv8DetectionClass, YOLOv8Node> currentlyDetectedNodes = new HashMap<>();
   private final Map<YOLOv8DetectionClass, YOLOv8SegmentedDetection> detectedObjects = new ConcurrentHashMap<>();
   private final Map<YOLOv8DetectionClass, YOLOv8SegmentedDetection> candidateDetections = new ConcurrentHashMap<>();

   private final IHMCROS2Input<YOLOv8ParametersMessage> yoloParameterSubscription;
   private float yoloConfidenceThreshold = 0.5f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;
   private float candidateAcceptanceThreshold = 0.6f;

   private ReferenceFrame robotFrame = null;

   public YOLOv8DetectionManager(ROS2Helper ros2Helper)
   {
      yoloDetectionThread = new RestartableThrottledThread("YOLODetector", 5.0, this::runYOLODetection);
      yoloDetectionThread.start();

      readyToRunNotification.set();

      yoloParameterSubscription = ros2Helper.subscribe(PerceptionAPI.YOLO_PARAMETERS);
      yoloParameterSubscription.addCallback(parametersMessage ->
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
      yoloDetectionThread.blockingStop();
      segmenter.destroy();
      extractor.destroy();
      openCLManager.destroy();

      yoloDetector.destroy();
      if (colorImage != null)
         colorImage.release();
      if (depthImage != null)
         depthImage.release();
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
         colorImageCopy = colorImage.get();
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

      if (colorImageCopy != null && depthImageCopy != null && robotFrame != null)
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

      PerceptionDebugTools.display("Color", colorImage.getCpuImageMat(), 1);
      YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(colorImage, yoloConfidenceThreshold, yoloNMSThreshold);

      // Extract stuff from the results
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getSegmentationImages(yoloSegmentationThreshold);
      Set<YOLOv8Detection> newDetections = yoloResults.getDetections();

      // match new detections to existing detections
      Point3D robotPoint = new Point3D(robotFrame.getTransformToWorldFrame().getTranslation());
      for (YOLOv8SegmentedDetection oldDetecion : detectedObjects.values())
      {
         double closestDistance = Double.POSITIVE_INFINITY;
         YOLOv8SegmentedDetection bestMatchDetection = null;

         Iterator<YOLOv8Detection> newDetectionIterator = newDetections.iterator();
         while (newDetectionIterator.hasNext())
         {
            YOLOv8Detection newDetection = newDetectionIterator.next();

            if (newDetection.objectClass() == oldDetecion.getDetection().objectClass())
            {
               RawImage mask = objectMasks.get(newDetection);

               YOLOv8SegmentedDetection segmentedNewDetection = new YOLOv8SegmentedDetection(newDetection,
                                                                                             oldDetecion.getDetectionFilter(),
                                                                                             mask,
                                                                                             depthImage,
                                                                                             segmenter::removeBackground,
                                                                                             extractor::extractPointCloud);

               double distance = robotPoint.distanceSquared(segmentedNewDetection.getCentroid());
               if (distance < closestDistance)
               {
                  closestDistance = distance;
                  bestMatchDetection = segmentedNewDetection;
               }

               newDetectionIterator.remove();
            }
         }

         if (bestMatchDetection != null)
         {
            detectedObjects.replace(bestMatchDetection.getDetection().objectClass(), bestMatchDetection);
            DetectionFilter filter = bestMatchDetection.getDetectionFilter();
            filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
            filter.registerDetection();
            filter.update();
         }
      }

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
                                                                                             segmenter::removeBackground,
                                                                                             extractor::extractPointCloud);

               double distance = robotPoint.distanceSquared(segmentedNewDetection.getCentroid());
               if (distance < closestDistance)
               {
                  closestDistance = distance;
                  bestMatchDetection = segmentedNewDetection;
               }

               newDetectionIterator.remove();
            }
         }

         if (bestMatchDetection != null)
         {
            candidateDetections.replace(bestMatchDetection.getDetection().objectClass(), bestMatchDetection);
            DetectionFilter filter = bestMatchDetection.getDetectionFilter();
            filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
            filter.registerDetection();
            filter.update();
         }
      }

      // Only newly detected object types left; add to candidate detections
      for (YOLOv8Detection newDetection : newDetections)
      {
         DetectionFilter filter = new DetectionFilter((int) yoloFrequencyCalculator.getFrequency(), candidateAcceptanceThreshold);
         filter.registerDetection();
         filter.update();

         YOLOv8SegmentedDetection segmentedNewDetection = new YOLOv8SegmentedDetection(newDetection,
                                                                                       filter,
                                                                                       objectMasks.get(newDetection),
                                                                                       depthImage,
                                                                                       segmenter::removeBackground,
                                                                                       extractor::extractPointCloud);

         candidateDetections.put(newDetection.objectClass(), segmentedNewDetection);
      }

      colorImage.release();
      depthImage.release();

      for (RawImage mask : objectMasks.values())
         mask.release();
      yoloResults.destroy();

      System.out.println("Run Frequency: " + yoloFrequencyCalculator.getFrequency());

      readyToRunNotification.set();
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         // Handle existing detections
         Iterator<Entry<YOLOv8DetectionClass, YOLOv8Node>> yoloNodeIterator = currentlyDetectedNodes.entrySet().iterator();
         while (yoloNodeIterator.hasNext())
         {
            YOLOv8Node yoloNode = yoloNodeIterator.next().getValue();
            YOLOv8SegmentedDetection detection = detectedObjects.get(yoloNode.getDetection().objectClass());
            DetectionFilter filter = detection.getDetectionFilter();

            if (filter.isStableDetectionResult() && sceneGraph.getIDToNodeMap().containsKey(yoloNode.getID()))
            {
               yoloNode.setObjectPointCloud(detection.getObjectPointCloud());
               yoloNode.update();
            }
            else
            {
               modificationQueue.accept(new SceneGraphNodeRemoval(yoloNode, sceneGraph));
               yoloNodeIterator.remove();
               detectedObjects.remove(detection.getDetection().objectClass());
            }
         }

         // Handle candidate detections (add or forget)
         Iterator<Entry<YOLOv8DetectionClass, YOLOv8SegmentedDetection>> candidateIterator = candidateDetections.entrySet().iterator();
         while (candidateIterator.hasNext())
         {
            YOLOv8SegmentedDetection candidateDetection = candidateIterator.next().getValue();
            DetectionFilter filter = candidateDetection.getDetectionFilter();

            if (filter.hasEnoughSamples())
            {
               if (filter.isStableDetectionResult())
               {
                  long nodeID = sceneGraph.getNextID().getAndIncrement();
                  YOLOv8Node newYoloNode = new YOLOv8Node(nodeID,
                                                          "YOLO " + candidateDetection.getDetection().objectClass().toString(),
                                                          candidateDetection.getDetection());
                  modificationQueue.accept(new SceneGraphNodeAddition(newYoloNode, sceneGraph.getRootNode()));
                  currentlyDetectedNodes.put(candidateDetection.getDetection().objectClass(), newYoloNode);
                  detectedObjects.put(candidateDetection.getDetection().objectClass(), candidateDetection);
                  candidateDetection.getDetectionFilter().setAcceptanceThreshold(0.2f);
               }

               candidateIterator.remove();
            }
         }
      });
   }
}

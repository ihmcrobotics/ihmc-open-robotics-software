package us.ihmc.perception.sceneGraph.yolo;

import perception_msgs.msg.dds.YOLOv8ParametersMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8DetectableObject;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.tools.thread.RestartableThread;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.Iterator;
import java.util.List;
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

   private final RestartableThread yoloDetectionThread;
   private final Lock newImageLock = new ReentrantLock();
   private final Condition newImageAvailable = newImageLock.newCondition();
   private final Notification readyToRunNotification = new Notification();
   private final FrequencyCalculator yoloFrequencyCalculator = new FrequencyCalculator();

   private final Map<YOLOv8Detection, DetectionFilter> candidateDetections = new ConcurrentHashMap<>();
   private final Map<YOLOv8DetectableObject, YOLOv8Node> currentlyDetectedNodes = new ConcurrentHashMap<>();

   private final ROS2Helper ros2Helper;
   private final IHMCROS2Input<YOLOv8ParametersMessage> yoloParameterSubscription;
   private float yoloConfidenceThreshold = 0.3f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;
   private float candidateAcceptanceThreshold = 0.6f;

   public YOLOv8DetectionManager(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;

      yoloDetectionThread = new RestartableThread("YOLODetector", this::runYOLODetection);

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
      if (colorImage != null)
         colorImage.release();
      colorImage = newColorImage.get();

      if (depthImage != null)
         depthImage.release();
      depthImage = newDepthImage.get();
   }

   public void updateSceneGraph(ROS2SceneGraph sceneGraph)
   {
      sceneGraph.modifyTree(modificationQueue ->
      {
         // Handle existing detections
         for (YOLOv8Node yoloNode : currentlyDetectedNodes.values())
         {
            if (!yoloNode.update())
               modificationQueue.accept(new SceneGraphNodeRemoval(yoloNode, sceneGraph));
         }

         // Handle candidate detections (add or forget)
//         synchronized (candidateDetections)
//         {
            Iterator<Entry<YOLOv8Detection, DetectionFilter>> candidateIterator = candidateDetections.entrySet().iterator();
            while (candidateIterator.hasNext())
            {
               Map.Entry<YOLOv8Detection, DetectionFilter> candidateDetectionEntry = candidateIterator.next();
               YOLOv8Detection candidateDetection = candidateDetectionEntry.getKey();
               DetectionFilter filter = candidateDetectionEntry.getValue();

               if (filter.hasEnoughSamples())
               {
                  if (filter.isStableDetectionResult())
                  {
                     long nodeID = sceneGraph.getNextID().getAndIncrement();
                     YOLOv8Node newYOLOICPCombo = new YOLOv8Node(nodeID, "YOLO " + candidateDetection.objectClass().name(), candidateDetection, filter);
                     modificationQueue.accept(new SceneGraphNodeAddition(newYOLOICPCombo, sceneGraph.getRootNode()));
                     sceneGraph.getIDToNodeMap().put(nodeID, newYOLOICPCombo);
                     currentlyDetectedNodes.put(candidateDetection.objectClass(), newYOLOICPCombo);
                  }

                  candidateIterator.remove();
               }
            }
//         }
      });
   }

   public void start()
   {
      yoloDetectionThread.start();
   }

   public void destroy()
   {
      yoloDetectionThread.stop();
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

      if (colorImageCopy != null && depthImageCopy != null)
      {
         if (readyToRunNotification.poll())
            runYOLOAndSegmentation(colorImageCopy, depthImageCopy);
         colorImageCopy.release();
         depthImageCopy.release();
      }
   }

   private void runYOLOAndSegmentation(RawImage colorImage, RawImage depthImage)
   {
      colorImage.get();
      depthImage.get();

      yoloFrequencyCalculator.ping();

      YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(colorImage, yoloConfidenceThreshold, yoloNMSThreshold);

      // Extract stuff from the results
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getSegmentationImages(yoloSegmentationThreshold);
      Set<YOLOv8Detection> newDetections = yoloResults.getDetections();

      // TODO: Does this need to be synchronized?
      matchDetections(currentlyDetectedNodes, newDetections);

      // Provide segmented point cloud to each yolo node
      for (YOLOv8Node yoloNode : currentlyDetectedNodes.values())
      {
         RawImage mask = objectMasks.get(yoloNode.getYoloDetection());
         if (mask != null)
         {
            RawImage segmentedDepth = segmenter.removeBackground(depthImage, mask, yoloNode.getMaskErosionKernelRadius());
            List<Point3DReadOnly> objectPointCloud = extractor.extractPointCloud(segmentedDepth);
            yoloNode.setObjectPointCloud(objectPointCloud);
         }
      }

      colorImage.release();
      depthImage.release();

      for (RawImage mask : objectMasks.values())
         mask.release();
      yoloResults.destroy();

      readyToRunNotification.set();
   }

   private void matchDetections(Map<YOLOv8DetectableObject, YOLOv8Node> oldDetections, Set<YOLOv8Detection> newDetections)
   {
      // Find best matching new detection for the old detections
      for (Entry<YOLOv8DetectableObject, YOLOv8Node> oldDetectionNode : oldDetections.entrySet())
      {
         YOLOv8Node yoloNode = oldDetectionNode.getValue();
         YOLOv8Detection oldDetection = yoloNode.getYoloDetection();

         // Find the new detection with the shortest distance between it and the old detection
         double bestMatchDistance = Double.POSITIVE_INFINITY;
         YOLOv8Detection bestMatchDetection = null;

         Iterator<YOLOv8Detection> newDetectionIterator = newDetections.iterator();
         while (newDetectionIterator.hasNext())
         {
            YOLOv8Detection newDetection = newDetectionIterator.next();
            if (newDetection.objectClass() == oldDetection.objectClass())
            {
               double distanceSquared = MathTools.square(oldDetection.x() - newDetection.x()) + MathTools.square(oldDetection.y() - newDetection.y());

               if (distanceSquared < bestMatchDistance)
               {
                  bestMatchDistance = distanceSquared;
                  bestMatchDetection = newDetection;
               }

               // remove new detections which are of same object type as old detection
               newDetectionIterator.remove();
            }
         }

         if (bestMatchDetection != null)
         {
            DetectionFilter filter = yoloNode.getDetectionFilter();
            filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
            filter.registerDetection();
            filter.update();
            yoloNode.setYoloDetection(bestMatchDetection);
         }
      }

      // Only undetected object types left now; match to candidate detections
      for (Entry<YOLOv8Detection, DetectionFilter> candidateEntry : candidateDetections.entrySet())
      {
         YOLOv8Detection candidateDetection = candidateEntry.getKey();

         double bestMatchDistance = Double.POSITIVE_INFINITY;
         YOLOv8Detection bestMatchDetection = null;

         Iterator<YOLOv8Detection> newDetectionIterator = newDetections.iterator();
         while (newDetectionIterator.hasNext())
         {
            YOLOv8Detection newDetection = newDetectionIterator.next();
            if (newDetection.objectClass() == candidateDetection.objectClass())
            {
               double distanceSquared =
                     MathTools.square(candidateDetection.x() - newDetection.x()) + MathTools.square(candidateDetection.y() - newDetection.y());

               if (distanceSquared < bestMatchDistance)
               {
                  bestMatchDistance = distanceSquared;
                  bestMatchDetection = newDetection;
               }

               // remove new detections which are of same object type as candidate detection
               newDetectionIterator.remove();
            }
         }

         if (bestMatchDetection != null)
         {
            DetectionFilter filter = candidateDetections.remove(candidateDetection);
            filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
            filter.registerDetection();
            filter.update();
            candidateDetections.put(bestMatchDetection, filter);
         }
      }

      // Only newly detected object types left; add to candidate detections
      for (YOLOv8Detection newDetection : newDetections)
      {
         DetectionFilter filter = new DetectionFilter((int) yoloFrequencyCalculator.getFrequency(), candidateAcceptanceThreshold);
         filter.registerDetection();
         filter.update();
         candidateDetections.put(newDetection, filter);
      }
   }
}

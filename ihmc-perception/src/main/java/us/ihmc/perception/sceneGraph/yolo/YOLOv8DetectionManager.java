package us.ihmc.perception.sceneGraph.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class YOLOv8DetectionManager
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final double FONT_SCALE = 1.5;
   private static final int FONT_THICKNESS = 2;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
   private static final Scalar BOUNDING_BOX_COLOR = new Scalar(0.0, 196.0, 0.0, 255.0);

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final ROS2DemandGraphNode annotatedImageDemandNode;
   private final ROS2PublisherBasics<ImageMessage> annotatedImagePublisher;

   private final YOLOv8ObjectDetector yoloDetector = new YOLOv8ObjectDetector();
   private final ExecutorService yoloExecutorService = Executors.newCachedThreadPool(ThreadTools.createNamedThreadFactory("YOLOExecutor"));
   private final FrequencyCalculator yoloFrequencyCalculator = new FrequencyCalculator();

   private final Map<YOLOv8DetectionClass, YOLOv8Node> detectedNodes = new ConcurrentHashMap<>();
   private final Map<YOLOv8DetectionClass, YOLOv8SegmentedDetection> detectedObjects = new ConcurrentHashMap<>();
   private final Map<YOLOv8DetectionClass, YOLOv8SegmentedDetection> candidateDetections = new ConcurrentHashMap<>();

   private float yoloConfidenceThreshold = 0.5f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;
   private float candidateAcceptanceThreshold = 0.6f;

   private Set<YOLOv8DetectionClass> targetDetections = new HashSet<>();

   private ReferenceFrame robotFrame = null;

   public YOLOv8DetectionManager(ROS2Helper ros2Helper, ROS2DemandGraphNode annotatedImageDemandNode)
   {
      this.annotatedImageDemandNode = annotatedImageDemandNode;

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "yolo_detection_manager");
      annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);

      ros2Helper.subscribe(PerceptionAPI.YOLO_PARAMETERS).addCallback(parametersMessage ->
      {
         yoloConfidenceThreshold = parametersMessage.getConfidenceThreshold();
         yoloNMSThreshold = parametersMessage.getNonMaximumSuppressionThreshold();
         yoloSegmentationThreshold = parametersMessage.getSegmentationThreshold();
         candidateAcceptanceThreshold = parametersMessage.getCandidateAcceptanceThreshold();

         // Create a new set of target detections to use
         Set<YOLOv8DetectionClass> newTargetDetections = new HashSet<>(parametersMessage.getTargetDetectionClasses().size());
         for (int i = 0; i < parametersMessage.getTargetDetectionClasses().size(); ++i)
            newTargetDetections.add(YOLOv8DetectionClass.fromByte(parametersMessage.getTargetDetectionClasses().get(i)));

         targetDetections = newTargetDetections;
      });
   }

   public void runYOLODetection(RawImage colorImage, RawImage depthImage)
   {
      if (yoloDetector.isReady() && !yoloExecutorService.isShutdown())
      {
         yoloExecutorService.submit(() ->
         {
            if (!colorImage.isAvailable() || !depthImage.isAvailable())
               return;
            colorImage.get();
            depthImage.get();

            YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(colorImage, yoloConfidenceThreshold, yoloNMSThreshold);
            segmentAndMatchDetections(yoloResults, depthImage, robotFrame);
            if (annotatedImageDemandNode.isDemanded())
               annotateAndPublishImage(yoloResults, colorImage);

            yoloResults.destroy();
            colorImage.release();
            depthImage.release();
         });
      }
   }

   public void setRobotFrame(ReferenceFrame robotFrame)
   {
      this.robotFrame = robotFrame;
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      shutdownExecutor();
      segmenter.destroy();
      extractor.destroy();
      openCLManager.destroy();

      yoloDetector.destroy();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private void shutdownExecutor()
   {
      yoloExecutorService.shutdown();
      try
      {
         if (!yoloExecutorService.awaitTermination(2, TimeUnit.SECONDS))
         {
            yoloExecutorService.shutdownNow();
            if (!yoloExecutorService.awaitTermination(2, TimeUnit.SECONDS))
               LogTools.error("YOLO executor failed to shutdown");
         }
      }
      catch (InterruptedException e)
      {
         yoloExecutorService.shutdownNow();
         LogTools.error(e);
      }
   }

   private void segmentAndMatchDetections(YOLOv8DetectionResults yoloResults, RawImage depthImage, ReferenceFrame robotFrame)
   {
      depthImage.get();

      yoloFrequencyCalculator.ping();

      // Extract stuff from the results
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getTargetSegmentationImages(yoloSegmentationThreshold, targetDetections);
      Set<YOLOv8Detection> newDetections = objectMasks.keySet();

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
            filter.registerDetection(bestMatchDetection.getDetection().confidence());
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
            filter.registerDetection(bestMatchDetection.getDetection().confidence());
         }

         filter.setHistoryLength((int) yoloFrequencyCalculator.getFrequency());
         filter.update();
      }

      // Only newly detected object classes left; add to candidate detections
      for (YOLOv8Detection newDetection : newDetections)
      {
         DetectionFilter filter = new DetectionFilter((int) yoloFrequencyCalculator.getFrequency(), candidateAcceptanceThreshold);
         filter.registerDetection(newDetection.confidence());
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

      for (RawImage mask : objectMasks.values())
         mask.release();
      depthImage.release();
   }

   public void annotateAndPublishImage(YOLOv8DetectionResults yoloResults, RawImage colorImage)
   {
      Mat resultMat = colorImage.get().getCpuImageMat().clone();

      Set<YOLOv8Detection> detections = yoloResults.getDetections();
      detections.stream().filter(detection -> detection.confidence() >= yoloConfidenceThreshold).forEach(detection ->
      {
         String text = String.format("%s: %.2f", detection.objectClass().toString(), detection.confidence());

         // Draw the bounding box
         Rect boundingBox = new Rect(detection.x(), detection.y(), detection.width(), detection.height());
         opencv_imgproc.rectangle(resultMat, boundingBox, BOUNDING_BOX_COLOR, 5, LINE_TYPE, 0);

         // Draw text background
         Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());
         Rect textBox = new Rect(detection.x(), detection.y() - textSize.height(), textSize.width(), textSize.height());
         opencv_imgproc.rectangle(resultMat, textBox, BOUNDING_BOX_COLOR, opencv_imgproc.FILLED, LINE_TYPE, 0);

         opencv_imgproc.putText(resultMat,
                                text,
                                new Point(detection.x(), detection.y()),
                                opencv_imgproc.CV_FONT_HERSHEY_DUPLEX,
                                FONT_SCALE,
                                new Scalar(255.0, 255.0, 255.0, 255.0),
                                FONT_THICKNESS,
                                LINE_TYPE,
                                false);
      });

      BytePointer annotatedImagePointer = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer); // for some reason using CUDAImageEncoder broke YOLO's CUDNN

      ImageMessage imageMessage = new ImageMessage();
      ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(annotatedImagePointer.limit());
      imageMessageDataPacker.pack(imageMessage, annotatedImagePointer);
      MessageTools.toMessage(colorImage.getAcquisitionTime(), imageMessage.getAcquisitionTime());
      imageMessage.setFocalLengthXPixels(colorImage.getFocalLengthX());
      imageMessage.setFocalLengthYPixels(colorImage.getFocalLengthY());
      imageMessage.setPrincipalPointXPixels(colorImage.getPrincipalPointX());
      imageMessage.setPrincipalPointYPixels(colorImage.getPrincipalPointY());
      imageMessage.setImageWidth(colorImage.getImageWidth());
      imageMessage.setImageHeight(colorImage.getImageHeight());
      imageMessage.getPosition().set(colorImage.getPosition());
      imageMessage.getOrientation().set(colorImage.getOrientation());
      imageMessage.setSequenceNumber(colorImage.getSequenceNumber());
      imageMessage.setDepthDiscretization(colorImage.getDepthDiscretization());
      CameraModel.PINHOLE.packMessageFormat(imageMessage);
      ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(imageMessage);

      annotatedImagePublisher.publish(imageMessage);

      resultMat.close();
      colorImage.release();
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
                                                          candidateDetection.getDetection().objectClass().getDefaultNodeName(),
                                                          candidateDetection.getDetection().objectClass(),
                                                          candidateDetection.getDetection().confidence(),
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
            yoloNode.setConfidence(detection.getDetection().confidence());
            yoloNode.update();
         }
      }
   }
}

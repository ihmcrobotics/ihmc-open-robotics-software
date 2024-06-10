package us.ihmc.perception.YOLOv8;

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
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.tools.thread.RestartableThread;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class YOLOv8DetectionManager
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final double FONT_SCALE = 1.5;
   private static final int FONT_THICKNESS = 2;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
   private static final Scalar BOUNDING_BOX_COLOR = new Scalar(0.0, 196.0, 0.0, 255.0);
   private static final int MAX_QUEUE_SIZE = 3;

   private final YOLOv8DetectionMatcher detectionMatcher = new YOLOv8DetectionMatcher();

   private final ROS2DemandGraphNode annotatedImageDemandNode;
   private final ROS2PublisherBasics<ImageMessage> annotatedImagePublisher;

   private final YOLOv8ObjectDetector yoloDetector = new YOLOv8ObjectDetector();
   private final ConcurrentLinkedQueue<YOLOv8DetectionResults> yoloResultQueue = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<RawImage> yoloDetectionColorImageQueue = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<RawImage> yoloDetectionDepthImageQueue = new ConcurrentLinkedQueue<>();
   private final ExecutorService yoloExecutorService = Executors.newCachedThreadPool(ThreadTools.createNamedThreadFactory("YOLOExecutor"));
   private final RestartableThread yoloProcessingThread;
   private final Lock newResultsLock = new ReentrantLock();
   private final Condition newResultsAvailable = newResultsLock.newCondition();

   private float yoloConfidenceThreshold = 0.5f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;
   private int erosionKernelRadius = 2;
   private float outlierRejectionThreshold = 0.2f;
   private float yoloMatchDistanceThreshold = 0.5f;
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
         erosionKernelRadius = 2;
         outlierRejectionThreshold = 0.2f;
         yoloMatchDistanceThreshold = 0.5f;
         candidateAcceptanceThreshold = parametersMessage.getCandidateAcceptanceThreshold();

         // Create a new set of target detections to use
         Set<YOLOv8DetectionClass> newTargetDetections = new HashSet<>(parametersMessage.getTargetDetectionClasses().size());
         for (int i = 0; i < parametersMessage.getTargetDetectionClasses().size(); ++i)
            newTargetDetections.add(YOLOv8DetectionClass.fromByte(parametersMessage.getTargetDetectionClasses().get(i)));

         targetDetections = newTargetDetections;
      });

      yoloProcessingThread = new RestartableThread("YOLODetectionProcessor", this::processYoloResults);
      yoloProcessingThread.start();
   }

   /**
    * Non-blocking call to run YOLO on the provided images
    * @param colorImage BGR color image, used for YOLO detection
    * @param depthImage 16UC1 depth image, used to get points of detected objects
    */
   public void runYOLODetection(RawImage colorImage, RawImage depthImage)
   {
      if (yoloDetector.isReady() && !yoloExecutorService.isShutdown())
      {
         yoloExecutorService.submit(() ->
         {
            // Acquire the images
            if (!colorImage.isAvailable() || !depthImage.isAvailable())
               return;
            colorImage.get();
            depthImage.get();

            // Run YOLO to get results
            YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(colorImage, yoloConfidenceThreshold, yoloNMSThreshold);

            // Put the results & images in queue to be processed further
            yoloResultQueue.add(yoloResults);
            yoloDetectionColorImageQueue.add(colorImage.get());
            yoloDetectionDepthImageQueue.add(depthImage.get());

            // Notify that new results came in
            newResultsLock.lock();
            try
            {
               newResultsAvailable.signal();
            }
            finally
            {
               newResultsLock.unlock();
            }

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
      yoloProcessingThread.stop();

      detectionMatcher.destroy();
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

   private void processYoloResults()
   {
      // Wait for new results if none are present
      newResultsLock.lock();
      try
      {
         while (yoloResultQueue.isEmpty() || yoloDetectionColorImageQueue.isEmpty() || yoloDetectionDepthImageQueue.isEmpty())
            newResultsAvailable.await();
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException);
      }
      finally
      {
         newResultsLock.unlock();
      }

      // Process existing results
      while (!yoloResultQueue.isEmpty() && !yoloDetectionDepthImageQueue.isEmpty() && !yoloDetectionColorImageQueue.isEmpty() && robotFrame != null)
      {
         // Discard old results if too many are present
         while (yoloResultQueue.size() > MAX_QUEUE_SIZE && yoloDetectionColorImageQueue.size() > MAX_QUEUE_SIZE && yoloDetectionDepthImageQueue.size() > MAX_QUEUE_SIZE)
         {
            yoloResultQueue.poll().destroy();
            yoloDetectionColorImageQueue.poll().release();
            yoloDetectionDepthImageQueue.poll().release();
         }

         // get the images & yolo results
         RawImage colorImage = yoloDetectionColorImageQueue.poll();
         RawImage depthImage = yoloDetectionDepthImageQueue.poll();
         YOLOv8DetectionResults yoloResults = yoloResultQueue.poll();

         // process the detections for scene graph stuff
         Map<YOLOv8Detection, RawImage> newDetections = yoloResults.getTargetSegmentationImages(yoloSegmentationThreshold, targetDetections);
         detectionMatcher.matchDetections(newDetections,
                                          depthImage,
                                          yoloMatchDistanceThreshold,
                                          erosionKernelRadius,
                                          outlierRejectionThreshold,
                                          candidateAcceptanceThreshold);

         // if demanded, publish an annotated image
         if (annotatedImageDemandNode.isDemanded())
            annotateAndPublishImage(yoloResults, colorImage);

         // release everything
         for (RawImage mask : newDetections.values())
            mask.release();
         yoloResults.destroy();
         colorImage.release();
         depthImage.release();
      }
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
      detectionMatcher.updateSceneGraph(sceneGraph);
   }
}

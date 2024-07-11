package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
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
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final double FONT_SCALE = 1.5;
   private static final int FONT_THICKNESS = 2;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
   private static final Scalar BOUNDING_BOX_COLOR = new Scalar(0.0, 196.0, 0.0, 255.0);
   private static final Mat GREEN_MAT = new Mat(1, 1, opencv_core.CV_8UC3, new Scalar(0.0, 255.0, 0.0, 255.0));

   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor();
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter();

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private final BooleanSupplier isDemandedSupplier;
   private final ROS2PublisherBasics<ImageMessage> annotatedImagePublisher;

   private final YOLOv8ObjectDetector yoloDetector = new YOLOv8ObjectDetector("IHMC_obj_seg_0.1.onnx");
   private final ExecutorService yoloExecutorService = Executors.newCachedThreadPool(ThreadTools.createNamedThreadFactory("YOLOExecutor"));

   private float yoloConfidenceThreshold = 0.5f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloSegmentationThreshold = 0.0f;
   private int erosionKernelRadius = 2;
   private double outlierThreshold = 1.0;

   private Set<YOLOv8DetectionClass> targetDetections = new HashSet<>();

   public YOLOv8DetectionExecutor(ROS2Helper ros2Helper, BooleanSupplier isDemandedSupplier)
   {
      this.isDemandedSupplier = isDemandedSupplier;

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "yolo_detection_manager");
      annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);

      ros2Helper.subscribe(PerceptionAPI.YOLO_PARAMETERS).addCallback(parametersMessage ->
      {
         yoloConfidenceThreshold = parametersMessage.getConfidenceThreshold();
         yoloNMSThreshold = parametersMessage.getNonMaximumSuppressionThreshold();
         yoloSegmentationThreshold = parametersMessage.getSegmentationThreshold();
         erosionKernelRadius = parametersMessage.getErosionKernelRadius();
         outlierThreshold = parametersMessage.getOutlierThreshold();

         // Create a new set of target detections to use
         Set<YOLOv8DetectionClass> newTargetDetections = new HashSet<>(parametersMessage.getTargetDetectionClasses().size());
         for (int i = 0; i < parametersMessage.getTargetDetectionClasses().size(); ++i)
            newTargetDetections.add(YOLOv8DetectionClass.fromByte(parametersMessage.getTargetDetectionClasses().get(i)));

         targetDetections = newTargetDetections;
      });
   }

   public void addDetectionConsumerCallback(Consumer<List<InstantDetection>> callback)
   {
      detectionConsumerCallbacks.add(callback);
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

            // Get the object masks from the results
            Map<YOLOv8DetectionOutput, RawImage> simpleDetectionMap = yoloResults.getTargetSegmentationImages(yoloSegmentationThreshold, targetDetections);

            // Create list of instant detections from results
            List<InstantDetection> yoloInstantDetections = new ArrayList<>();
            for (Map.Entry<YOLOv8DetectionOutput, RawImage> simpleDetectionEntry : simpleDetectionMap.entrySet())
            {
               YOLOv8DetectionOutput simpleDetection = simpleDetectionEntry.getKey();
               RawImage objectMask = simpleDetectionEntry.getValue();

               // Erode mask to get better segmentation
               Mat erodedMask = new Mat(objectMask.getImageHeight(), objectMask.getImageWidth(), objectMask.getOpenCVType());
               opencv_imgproc.erode(objectMask.getCpuImageMat(),
                                    erodedMask,
                                    opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                         new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                         new Point(erosionKernelRadius, erosionKernelRadius)));
               erodedMask.copyTo(objectMask.getCpuImageMat()); // RawImage isn't designed for this, but it works... 10/10 wouldn't recommend.
               objectMask.getGpuImageMat().upload(erodedMask);

               // Get the segmented depth image
               RawImage segmentedDepth = segmenter.removeBackground(depthImage, objectMask);
               // Get the point cloud
               List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);
               // Filter out outliers from the point cloud
               pointCloud = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);
               // Get the centroid of the point cloud
               Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(pointCloud, 128);
               if (centroid.containsNaN())
                  return;

               // Create an instant detection from data
               YOLOv8InstantDetection instantDetection = new YOLOv8InstantDetection(simpleDetection.objectClass().getDefaultNodeName(),
                                                                                    simpleDetection.confidence(),
                                                                                    new Pose3D(centroid, new RotationMatrix()),
                                                                                    objectMask.getAcquisitionTime(),
                                                                                    pointCloud);
               yoloInstantDetections.add(instantDetection);
            }

            // Submit the callbacks to be processed
            yoloExecutorService.submit(() -> detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections)));

            // If annotated image is demanded, create and publish it
            if (isDemandedSupplier.getAsBoolean())
               annotateAndPublishImage(yoloResults, colorImage);

            yoloResults.destroy();
            colorImage.release();
            depthImage.release();
         });
      }
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      shutdownExecutor();
      segmenter.destroy();
      extractor.destroy();

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

   public void annotateAndPublishImage(YOLOv8DetectionResults yoloResults, RawImage colorImage)
   {
      Mat resultMat = colorImage.get().getCpuImageMat().clone();

      Map<YOLOv8DetectionOutput, RawImage> detectionMasks = yoloResults.getTargetSegmentationImages(yoloSegmentationThreshold, targetDetections);
      detectionMasks.entrySet().stream().filter(entry -> entry.getKey().confidence() >= yoloConfidenceThreshold).forEach(entry ->
      {
         YOLOv8DetectionOutput detection = entry.getKey();
         RawImage maskImage = entry.getValue();

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

         // Add green tint to show mask
         // first convert 32F mask to 8U
         Mat maskMat = new Mat(maskImage.getImageHeight(), maskImage.getImageWidth(), opencv_core.CV_8U);
         maskImage.getCpuImageMat().convertTo(maskMat, opencv_core.CV_8U, 255.0, 0.0);

         // resize the mask to fit the result image
         opencv_imgproc.resize(maskMat, maskMat, resultMat.size(), 0.0, 0.0, opencv_imgproc.INTER_NEAREST);

         // ensure the green Mat is same size as image
         if (resultMat.cols() != GREEN_MAT.cols() || resultMat.rows() != GREEN_MAT.rows())
            opencv_imgproc.resize(GREEN_MAT, GREEN_MAT, resultMat.size());

         // add a green tint where mask = 255
         opencv_core.add(resultMat, GREEN_MAT, resultMat, maskMat, -1);

         maskImage.release();
         maskMat.release();
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
}

package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor();
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter();

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private final BooleanSupplier annotatedImageDemanded;
   private final ROS2Publisher<ImageMessage> annotatedImagePublisher;

   // TODO: temp hack
   private int lastRunModelIndex = 0;
   private final List<YOLOv8Model> yoloModels = new ArrayList<>();
   private final BlockingQueue<Runnable> taskQueue;
   private final RepeatingTaskThread taskExecutorThread = new RepeatingTaskThread("YOLOExecutor", this::executeTasks, DefaultExceptionHandler.RUNTIME_EXCEPTION);

   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final Map<YOLOv8Model, YOLOv8DetectionList> yoloDetectionResults = new ConcurrentHashMap<>();
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   private float yoloConfidenceThreshold = 0.5f;
   private float yoloNMSThreshold = 0.1f;
   private float yoloMaskThreshold = 0.0f;
   private int erosionKernelRadius = 2;
   private double outlierThreshold = 1.0;

   public YOLOv8DetectionExecutor(ROS2Helper ros2Helper, BooleanSupplier annotatedImageDemanded)
   {
      this.annotatedImageDemanded = annotatedImageDemanded;

      ROS2Node ros2Node = new ROS2NodeBuilder().build("yolo_detection_manager");
      annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);

      ros2Helper.subscribe(PerceptionAPI.YOLO_PARAMETERS).addCallback(parametersMessage ->
      {
         yoloConfidenceThreshold = parametersMessage.getConfidenceThreshold();
         yoloNMSThreshold = parametersMessage.getNonMaximumSuppressionThreshold();
         yoloMaskThreshold = parametersMessage.getSegmentationThreshold();
         erosionKernelRadius = parametersMessage.getErosionKernelRadius();
         outlierThreshold = parametersMessage.getOutlierThreshold();
      });

      for (Path yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: " + YOLOv8Tools.getONNXFile(yoloModelDirectory));
         LogTools.info("\t\t\tClasses: " + model.getDetectionClassNames().size());

         yoloModels.add(model);
      }

      if (yoloModels.isEmpty())
         LogTools.error("No YOLO models found. YOLO will not run.");

      taskQueue = new ArrayBlockingQueue<>(2 * yoloModels.size());
      taskExecutorThread.setDaemon(true);
      taskExecutorThread.startRepeating();

      annotatedImagePublishedThread = new RepeatingTaskThread("YOLOAnnotatedImagePublisher", this::annotateAndPublishImage, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      annotatedImagePublishedThread.setDaemon(true);
      annotatedImagePublishedThread.startRepeating();
   }

   public void addDetectionConsumerCallback(Consumer<List<InstantDetection>> callback)
   {
      detectionConsumerCallbacks.add(callback);
   }

   public void runYOLODetectionOnAllModels(RawImage colorImage, RawImage depthImage)
   {
      if (yoloModels.isEmpty())
         return;

      if (lastRunModelIndex + 1 > yoloModels.size())
         lastRunModelIndex = 0;

      YOLOv8Model yoloModel = yoloModels.get(lastRunModelIndex++);

      runYOLODetection(yoloModel, colorImage, depthImage);
   }

   /**
    * Non-blocking call to run YOLO on the provided images
    * @param colorImage BGR color image, used for YOLO detection
    * @param depthImage 16UC1 depth image, used to get points of detected objects
    */
   public void runYOLODetection(YOLOv8Model yoloModel, RawImage colorImage, RawImage depthImage)
   {
      if (taskQueue.remainingCapacity() > 0)
      {
         // Acquire the images
         if (colorImage.get() == null || depthImage.get() == null)
            return;

         taskQueue.add(() ->
         {
            // Run YOLO to get results
            GpuMat bgrMat = new GpuMat();
            colorImage.getPixelFormat().convertToPixelFormat(colorImage.getGpuImageMat(), bgrMat, PixelFormat.BGR8);
            RawImage bgrImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
            YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage, yoloConfidenceThreshold, yoloNMSThreshold, yoloMaskThreshold);

            // TODO: temp hack
            synchronized (yoloDetectionResults)
            {
               if (yoloDetectionResults.containsKey(yoloModel))
                  yoloDetectionResults.remove(yoloModel).destroy();
               yoloDetectionResults.put(yoloModel, yoloResults);
            }

            if (newestColorImage.poll())
               newestColorImage.read().release();
            newestColorImage.set(bgrImage.get());

            // Create list of instant detections from results
            List<InstantDetection> yoloInstantDetections = new ArrayList<>();
            for (YOLOv8Detection detection : yoloResults)
            {
               RawImage objectMask = detection.mask();

               // Erode mask to get better segmentation
               Mat erodedMask = new Mat(objectMask.getHeight(), objectMask.getWidth(), objectMask.getOpenCVType());
               opencv_imgproc.erode(objectMask.getCpuImageMat(),
                                    erodedMask,
                                    opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                         new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                         new Point(erosionKernelRadius, erosionKernelRadius)));
               RawImage erodedObjectMask = objectMask.replaceImage(erodedMask);

               // Get the segmented depth image
               RawImage segmentedDepth = segmenter.removeBackground(depthImage, erodedObjectMask);
               // Get the point cloud
               List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);
               // Filter out outliers from the point cloud
               pointCloud = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);
               // Get the centroid of the point cloud
               Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(pointCloud, 128);
               if (centroid.containsNaN())
                  return;

               // Create an instant detection from data
               YOLOv8InstantDetection instantDetection = new YOLOv8InstantDetection(detection.objectClass(),
                                                                                    detection.confidence(),
                                                                                    new Pose3D(centroid, new RotationMatrix()),
                                                                                    objectMask.getAcquisitionTime(),
                                                                                    bgrImage,
                                                                                    erodedObjectMask,
                                                                                    depthImage,
                                                                                    pointCloud);
               yoloInstantDetections.add(instantDetection);
               erodedMask.release();
            }

            // Process callbacks
            if (!yoloInstantDetections.isEmpty())
               detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections));

            bgrImage.release();
            colorImage.release();
            depthImage.release();
         });
      }
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      taskExecutorThread.kill();
      annotatedImagePublishedThread.kill();
      newestColorImage.set(null);

      segmenter.destroy();
      extractor.destroy();

      for (YOLOv8Model yoloModel : yoloModels)
         yoloModel.destroy();

      for (YOLOv8DetectionList yoloResults : yoloDetectionResults.values())
         yoloResults.destroy();

      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private void executeTasks()
   {
      try
      {
         taskQueue.take().run();
      }
      catch (InterruptedException ignored) {}
   }

   private void annotateAndPublishImage()
   {
      if (!annotatedImageDemanded.getAsBoolean())
         return;

      RawImage colorImage = newestColorImage.blockingPoll();
      if (colorImage == null)
         return;

      Mat resultMat = new Mat();
      synchronized (yoloDetectionResults)
      {
         List<YOLOv8Detection> allDetections = new ArrayList<>();
         yoloDetectionResults.values().forEach(allDetections::addAll);
         YOLOv8Tools.annotateImage(colorImage.getCpuImageMat(), resultMat, allDetections);
      }

      BytePointer annotatedImagePointer = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer); // for some reason using CUDAImageEncoder broke YOLO's CUDNN

      ImageMessage imageMessage = new ImageMessage();
      PerceptionMessageTools.packImageMessage(colorImage, annotatedImagePointer, CompressionType.JPEG, imageMessage);
      annotatedImagePublisher.publish(imageMessage);

      resultMat.close();
      colorImage.release();
   }
}

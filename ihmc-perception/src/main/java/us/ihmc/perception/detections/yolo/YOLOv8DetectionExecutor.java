package us.ihmc.perception.detections.yolo;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.YOLOv8AvailableModels;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("yolo_detection_manager");

   private final CUDAPointCloudExtractor extractor = new CUDAPointCloudExtractor();
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter();

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private final Map<String, YOLOv8Model> enabledModels = new LinkedHashMap<>();
   private final Map<YOLOv8Model, YOLOv8DetectionList> yoloDetectionResults = new ConcurrentHashMap<>();
   private Iterator<YOLOv8Model> modelIterator;

   private final Map<YOLOv8Model, TIntArrayList> erosionKernelRadiiMap = new HashMap<>();
   private final Map<YOLOv8Model, TFloatArrayList> outlierThresholdsMap = new HashMap<>();

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private final ROS2Publisher<ImageMessage> annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);
   private final BooleanSupplier annotatedImageDemanded;
   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   private final BlockingQueue<Runnable> taskQueue;
   private final RepeatingTaskThread taskExecutorThread = new RepeatingTaskThread("YOLOExecutor", this::executeTasks, DefaultExceptionHandler.RUNTIME_EXCEPTION);

   public YOLOv8DetectionExecutor(BooleanSupplier annotatedImageDemanded)
   {
      this.annotatedImageDemanded = annotatedImageDemanded;

      // Read available YOLO models
      for (Path yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: " + YOLOv8Tools.getONNXFile(yoloModelDirectory));
         LogTools.info("\t\t\tClasses: " + model.getDetectableObjectCount());

         availableModels.put(model.getName(), model);

         int[] defaultErosionKernelRadii = new int[model.getDetectableObjectCount()];
         Arrays.fill(defaultErosionKernelRadii, 2);
         erosionKernelRadiiMap.put(model, new TIntArrayList(defaultErosionKernelRadii));

         float[] defaultOutlierThresholds = new float[model.getDetectableObjectCount()];
         Arrays.fill(defaultOutlierThresholds, 1.0f);
         outlierThresholdsMap.put(model, new TFloatArrayList(defaultOutlierThresholds));
      }

      if (availableModels.isEmpty())
         LogTools.error("No YOLO models found. YOLO will not run.");

      // Create the available models message
      YOLOv8AvailableModels availableModelsMessage = new YOLOv8AvailableModels();
      availableModelsMessage.setRequest(false);
      for (YOLOv8Model model : availableModels.values())
      {
         YOLOv8ModelInfo modelInfo = availableModelsMessage.getAvailableYoloModels().add();
         modelInfo.setModelName(model.getName());
         model.getDetectableObjects().forEach(objectClass -> modelInfo.getDetectableObjectClasses().add(objectClass));
      }

      // Create a publisher for the message, and publish whenever a request is received
      ROS2Publisher<YOLOv8AvailableModels> availableModelsPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_AVAILABLE_MODELS);
      ros2Node.createSubscription2(PerceptionAPI.YOLO_AVAILABLE_MODELS, message ->
      {
         if (message.getRequest())
            availableModelsPublisher.publish(availableModelsMessage);
      });

      // Subscribe to YOLO settings messages
      ros2Node.createSubscription2(PerceptionAPI.YOLO_SETTINGS, settingsMessage ->
      {
         // Enable/disable models
         List<String> modelsToRun = settingsMessage.getModelsToRun().stream().map(StringBuilder::toString).toList();
         availableModels.keySet().forEach(model ->
         {
            if (modelsToRun.contains(model))
               enableModel(model);
            else
               disableModel(model);
         });

         // Update each model's settings according to the message
         settingsMessage.getModelSettings().forEach(modelSettings ->
         {
            YOLOv8Model model = availableModels.get(modelSettings.getModelNameAsString());
            boolean[] ignoredClasses = new boolean[modelSettings.getIgnoredObjectClasses().size()];
            for (int i = 0; i < modelSettings.getIgnoredObjectClasses().size(); ++i)
               ignoredClasses[i] = modelSettings.getIgnoredObjectClasses().getBoolean(i);
            model.setIgnoredClasses(ignoredClasses);
            model.setConfidenceThresholds(modelSettings.getConfidenceThresholds().toArray());
            model.setMaskThresholds(modelSettings.getMaskThresholds().toArray());
            model.setNMSThreshold(modelSettings.getNonMaximumSuppressionThreshold());

            TIntArrayList erosionKernelRadii = erosionKernelRadiiMap.get(model);
            erosionKernelRadii.set(0, modelSettings.getErosionKernelRadii().toArray());

            TFloatArrayList outlierThresholds = outlierThresholdsMap.get(model);
            outlierThresholds.set(0, modelSettings.getOutlierThresholds().toArray());
         });
      });

      taskQueue = new ArrayBlockingQueue<>(2 * availableModels.size());
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

   public List<String> getAvailableModelNames()
   {
      return new ArrayList<>(availableModels.keySet());
   }

   public void enableModel(String modelName)
   {
      enabledModels.put(modelName, availableModels.get(modelName));
   }

   public void enableAllModels()
   {
      enabledModels.putAll(availableModels);
   }

   public void disableModel(String modelName)
   {
      enabledModels.remove(modelName);
   }

   public void disableAllModels()
   {
      enabledModels.clear();
   }

   public void runNextEnabledModel(RawImage colorImage, RawImage depthImage)
   {
      if (modelIterator == null || !modelIterator.hasNext())
         modelIterator = availableModels.values().iterator();

      while (modelIterator.hasNext())
      {
         YOLOv8Model model = modelIterator.next();

         if (enabledModels.containsKey(model.getName()))
         {
            runYOLODetection(model, colorImage, depthImage);
            return;
         }

         yoloDetectionResults.remove(model);
      }
   }

   /**
    * Non-blocking call to run YOLO on the provided images
    * @param colorImage BGR color image, used for YOLO detection
    * @param depthImage 16UC1 depth image, used to get points of detected objects
    */
   private void runYOLODetection(YOLOv8Model yoloModel, RawImage colorImage, RawImage depthImage)
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
            YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage);

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
               int erosionKernelRadius = erosionKernelRadiiMap.get(yoloModel).get(detection.objectClassID());
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
               float outlierThreshold = outlierThresholdsMap.get(yoloModel).get(detection.objectClassID());
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
      ros2Node.destroy();

      taskExecutorThread.kill();
      taskExecutorThread.interrupt();

      annotatedImagePublishedThread.kill();
      newestColorImage.set(null);

      for (YOLOv8Model yoloModel : availableModels.values())
         yoloModel.destroy();

      for (YOLOv8DetectionList yoloResults : yoloDetectionResults.values())
         yoloResults.destroy();

      extractor.close();
      segmenter.destroy();

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

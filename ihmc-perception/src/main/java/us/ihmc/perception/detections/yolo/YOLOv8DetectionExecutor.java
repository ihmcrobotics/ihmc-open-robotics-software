package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudafilters;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_cudafilters.Filter;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.YOLOv8ExecutorParameters;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDADepthImageSegmenter;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;

import java.net.URL;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build("yolo_detection_manager");

   private final CUDAPointCloudExtractor extractor;
   private final CUDADepthImageSegmenter segmenter;

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private final TypedNotification<List<YOLOv8InstantDetection>> annotationNotification = new TypedNotification<>();

   private final SyncedYOLOv8ExecutorParameters parameters;
   private boolean requestingFullData;
   private final ROS2Publisher<YOLOv8ExecutorParameters> parametersPublisher;
   private final YOLOv8ExecutorParameters parametersMessage;
   private final RepeatingTaskThread updateThread;

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   private final ROS2Publisher<ImageMessage> annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);
   private final BooleanSupplier annotatedImageDemanded;
   private final RepeatingTaskThread annotatedImagePublishedThread;
   private final TypedNotification<RawImage> newestColorImage = new TypedNotification<>();

   public YOLOv8DetectionExecutor(ROS2PeerClockOffsetEstimator peerClockEstimator, BooleanSupplier annotatedImageDemanded)
   {
      this.annotatedImageDemanded = annotatedImageDemanded;

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);

      try
      {
         extractor = new CUDAPointCloudExtractor();
         segmenter = new CUDADepthImageSegmenter();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // Read available YOLO models
      for (URL yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: " + model.getName());
         LogTools.info("\t\t\tClasses: " + model.getDetectableObjectCount());

         availableModels.put(model.getName(), model);
      }

      if (availableModels.isEmpty())
         LogTools.error("No YOLO models found. YOLO will not run.");

      // Create YOLO parameters
      parameters = new SyncedYOLOv8ExecutorParameters(crdtInfo);
      parameters.setAvailableModels(availableModels.values());
      parameters.requestSendFullData();
      requestingFullData = true;

      // Subscribe to YOLO parameters messages
      ros2Node.createSubscription2(PerceptionAPI.YOLO_PARAMETERS, message ->
      {
         parameters.fromMessage(message);
         parameters.confirmReceivedFullData();
         requestingFullData = false;
      });

      parametersPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_PARAMETERS);
      parametersMessage = new YOLOv8ExecutorParameters();
      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Updater", this::update);
      updateThread.setFrequencyLimit(10.0);
      updateThread.setDaemon(true);
      updateThread.startRepeating();

      annotatedImagePublishedThread = new RepeatingTaskThread("YOLOAnnotatedImagePublisher",
                                                              this::annotateAndPublishImage,
                                                              DefaultExceptionHandler.RUNTIME_EXCEPTION);
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
      parameters.getModelToRun().setValue(modelName);
   }

   public void disableModel()
   {
      parameters.getModelToRun().setValue(null);
   }

   public void runModel(RawImage colorImage, RawImage depthImage)
   {
      // Acquire the images
      if (colorImage.get() == null || depthImage.get() == null)
         return;

      YOLOv8Model yoloModel = availableModels.get(parameters.getModelToRun().getValue());
      if (yoloModel == null)
         return;

      // Run YOLO to get results
      RawImage bgrImage = RawImageTools.convertColor(colorImage, PixelFormat.BGR8);
      YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage);

      if (newestColorImage.poll())
         newestColorImage.read().release();
      newestColorImage.set(bgrImage.get());

      SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters();

      // Create list of instant detections from results
      List<InstantDetection> yoloInstantDetections = new ArrayList<>();
      List<YOLOv8InstantDetection> annotatedImageDetections = new ArrayList<>();
      for (YOLOv8Detection detection : yoloResults)
      {
         RawImage objectMask = detection.mask();

         // Erode mask to get better segmentation
         RawImage erodedObjectMask = erodeMask(objectMask, modelParameters.getErosionKernelRadii().getValueReadOnly(detection.objectClassID()));
         objectMask.release();

         // Get the segmented depth image
         RawImage segmentedDepth = segmenter.removeBackground(depthImage, erodedObjectMask);
         if (segmentedDepth == null)
         {
            erodedObjectMask.release();
            continue;
         }

         // Get the point cloud
         List<Point3D32> pointCloud = extractor.extractPointCloud(segmentedDepth);
         // Filter out outliers from the point cloud
         float outlierThreshold = modelParameters.getOutlierThresholds().getValueReadOnly(detection.objectClassID());
         List<Point3D32> filteredPoints = YOLOv8Tools.filterOutliers(pointCloud, outlierThreshold, 128);

         if (!filteredPoints.isEmpty())
         {
            Point3D32 centroid = YOLOv8Tools.computeCentroidOfPointCloud(filteredPoints, 128);

            annotatedImageDetections.add(new YOLOv8InstantDetection(detection.objectClass(),
                                                                    detection.confidence(),
                                                                    new Pose3D(centroid, new RotationMatrix()),
                                                                    erodedObjectMask.getAcquisitionTime(),
                                                                    bgrImage,
                                                                    erodedObjectMask,
                                                                    depthImage,
                                                                    detection.boundingBox(),
                                                                    filteredPoints));
            yoloInstantDetections.add(new YOLOv8InstantDetection(detection.objectClass(),
                                                                 detection.confidence(),
                                                                 new Pose3D(centroid, new RotationMatrix()),
                                                                 erodedObjectMask.getAcquisitionTime(),
                                                                 bgrImage,
                                                                 erodedObjectMask,
                                                                 depthImage,
                                                                 detection.boundingBox(),
                                                                 filteredPoints));
         }
         erodedObjectMask.release();
         segmentedDepth.release();
      }

      // Process callbacks
      if (!yoloInstantDetections.isEmpty())
      {
         detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections));

         if (annotationNotification.poll())
            for (YOLOv8InstantDetection previousAnnotatedImageDetection : annotationNotification.read())
               previousAnnotatedImageDetection.destroy();

         annotationNotification.set(annotatedImageDetections);
      }

      yoloResults.destroy();
      bgrImage.release();
      colorImage.release();
      depthImage.release();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      updateThread.blockingKill();

      ros2Node.destroy();

      annotatedImagePublishedThread.kill();
      newestColorImage.set(null);

      for (YOLOv8Model yoloModel : availableModels.values())
         yoloModel.destroy();

      extractor.close();
      segmenter.close();

      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private RawImage erodeMask(RawImage objectMask, int erosionKernelRadius)
   {
      Mat erosionElement = opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                new Point(erosionKernelRadius, erosionKernelRadius));
      Filter erosionFilter = opencv_cudafilters.createMorphologyFilter(opencv_imgproc.MORPH_ERODE, opencv_core.CV_8U, erosionElement);

      GpuMat erodedMask = new GpuMat(objectMask.getHeight(), objectMask.getWidth(), objectMask.getOpenCVType());
      erosionFilter.apply(objectMask.getGpuImageMat(), erodedMask);

      erosionElement.close();
      erosionFilter.close();

      return objectMask.replaceImage(erodedMask);
   }

   private void annotateAndPublishImage()
   {
      annotationNotification.blockingPoll();
      List<YOLOv8InstantDetection> detectionsToAnnotate = annotationNotification.read();

      RawImage colorImage = detectionsToAnnotate.get(0).getColorImage().get();
      if (colorImage == null)
         return;

      if (!annotatedImageDemanded.getAsBoolean())
      {
         return;
      }

      Mat resultMat = new Mat();
      synchronized (annotatedImagePublishedThread)
      {
         YOLOv8Tools.annotateImage(colorImage.getCpuImageMat(), resultMat, detectionsToAnnotate);
      }

      BytePointer annotatedImagePointer = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer); // for some reason using CUDAImageEncoder broke YOLO's CUDNN

      ImageMessage imageMessage = new ImageMessage();
      PerceptionMessageTools.packImageMessage(colorImage, annotatedImagePointer, CompressionType.JPEG, imageMessage);
      annotatedImagePublisher.publish(imageMessage);

      resultMat.close();
      colorImage.release();
   }

   private void update()
   {
      parameters.checkModified();

      SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters();
      modelParameters.checkModified();

      if (modelParameters.isModified())
         modelParameters.applyToModel(availableModels.get(parameters.getModelToRun().getValue()));

      if (requestingFullData || parameters.pollNeedSendFullData() || modelParameters.pollNeedSendFullData())
      {
         parameters.toMessage(parametersMessage);
         parametersPublisher.publish(parametersMessage);
      }
   }
}

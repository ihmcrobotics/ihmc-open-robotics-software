package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.YOLOv8AnnotationInfoList;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
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
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.net.URL;
import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionExecutor
{
   private final CUDAPointCloudExtractor extractor;
   private final CUDADepthImageSegmenter segmenter;

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private final SyncedYOLOv8ExecutorParameters parameters;

   private final ROS2Publisher<YOLOv8AnnotationInfoList> annotationInfoPublisher;
   private final BooleanSupplier annotationInfoDemanded;

   private final RepeatingTaskThread updateThread;

   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   public void addDetectionConsumerCallback(Consumer<List<InstantDetection>> callback)
   {
      detectionConsumerCallbacks.add(callback);
   }

   public YOLOv8DetectionExecutor(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator, BooleanSupplier annotationInfoDemanded)
   {
      this.annotationInfoDemanded = annotationInfoDemanded;

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
      parameters = new SyncedYOLOv8ExecutorParameters(ros2Node, crdtInfo);
      parameters.setAvailableModels(availableModels.values());

      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Updater", this::update);
      updateThread.setFrequencyLimit(10.0);
      updateThread.setDaemon(true);
      updateThread.startRepeating();

      annotationInfoPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATION_INFO);
   }

   /**
    * Returns the names of all YOLOv8 models that are currently available to this executor.
    *
    * @return a new list containing the names of all loaded YOLOv8 models
    */
   public List<String> getAvailableModelNames()
   {
      return new ArrayList<>(availableModels.keySet());
   }

   /**
    * Selects the YOLOv8 model to run for subsequent detection calls.
    * If the specified model name does not correspond to a loaded model,
    * {@link #runModel(RawImage, RawImage)} will act as a no-op.
    *
    * @param modelName the name of the YOLOv8 model to enable, as returned by {@link #getAvailableModelNames()}
    */
   public void enableModel(String modelName)
   {
      parameters.getModelToRun().setValue(modelName);
   }

   /**
    * Disables the currently selected YOLOv8 model.
    * After calling this method, {@link #runModel(RawImage, RawImage)} will not perform detections
    * until a model is enabled again via {@link #enableModel(String)}.
    */
   public void disableModel()
   {
      parameters.getModelToRun().setValue(null);
   }

   /**
    * Runs the currently enabled YOLOv8 model on the given color and depth images, producing
    * 3D detections and invoking all registered detection callbacks.
    * <p>
    * If no model is enabled or either image is invalid, the method returns without producing detections.
    *
    * @param colorImage the raw color image to be used as input to YOLOv8 and for annotation
    * @param depthImage the raw depth image used to compute 3D information for each detection
    */
   public void runModel(RawImage colorImage, RawImage depthImage)
   {
      // Acquire the images
      if (colorImage.get() == null || depthImage.get() == null)
         return;

      List<InstantDetection> yoloInstantDetections = new ArrayList<>();

      YOLOv8Model yoloModel = availableModels.get(parameters.getModelToRun().getValue());
      if (yoloModel != null)
      {
         // Run YOLO to get results
         RawImage bgrImage = RawImageTools.convertColor(colorImage, PixelFormat.BGR8);
         YOLOv8DetectionList yoloResults = yoloModel.run(bgrImage);

         SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters();

         // Create list of instant detections from results
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

         yoloResults.destroy();
         bgrImage.release();
      }

      // Process callbacks
      if (!yoloInstantDetections.isEmpty())
         detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections));

      // Publish annotation information
      if (annotationInfoDemanded.getAsBoolean())
         publishAnnotationInfo(yoloInstantDetections);

      colorImage.release();
      depthImage.release();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      updateThread.blockingKill();

      for (YOLOv8Model yoloModel : availableModels.values())
         yoloModel.destroy();

      extractor.close();
      segmenter.close();

      parameters.close();

      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private RawImage erodeMask(RawImage objectMask, int erosionKernelRadius)
   {
      Mat erosionElement = opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                new Point(erosionKernelRadius, erosionKernelRadius));
      Mat erodedMask = new Mat(objectMask.getHeight(), objectMask.getWidth(), objectMask.getOpenCVType());
      opencv_imgproc.erode(objectMask.getCpuImageMat(), erodedMask, erosionElement);

      erosionElement.close();

      return objectMask.replaceImage(erodedMask);
   }

   private void publishAnnotationInfo(List<InstantDetection> detections)
   {
      YOLOv8AnnotationInfoList annotationInfo = new YOLOv8AnnotationInfoList();
      Instant detectionTime = null;

      for (InstantDetection detection : detections)
      {
         if (detection instanceof YOLOv8InstantDetection yoloInstantDetection)
         {
            if (detectionTime == null)
               detectionTime = yoloInstantDetection.getDetectionTime();

            YOLOv8AnnotationInfo.fromYOLOv8InstantDetection(yoloInstantDetection, 0.005f).toMessage(annotationInfo.getAnnotationInfos().add());
         }
      }

      if (detectionTime != null)
         MessageTools.toMessage(detectionTime, annotationInfo.getDetectionInstant());

      annotationInfoPublisher.publish(annotationInfo);
   }

   private void update()
   {
      parameters.checkModifiedAndUpdate();

      SyncedYOLOv8ModelParameters modelParameters = parameters.getModelParameters();
      modelParameters.checkModified();
      if (modelParameters.isModified())
         modelParameters.applyToModel(availableModels.get(parameters.getModelToRun().getValue()));
   }
}

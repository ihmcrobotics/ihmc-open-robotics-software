package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.ImageMessage;
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
   private final CUDAPointCloudExtractor extractor;
   private final CUDADepthImageSegmenter segmenter;

   private final Map<String, YOLOv8Model> availableModels = new LinkedHashMap<>();
   private final SyncedYOLOv8ExecutorParameters parameters;

   private final ROS2Publisher<ImageMessage> annotatedImagePublisher;
   private final BooleanSupplier annotatedImageDemanded;

   private final RepeatingTaskThread updateThread;
   private final RepeatingTaskThread annotatedImagePublishedThread;

   private final TypedNotification<YOLOv8AnnotationRecord> annotationNotification = new TypedNotification<>();
   private final List<Consumer<List<InstantDetection>>> detectionConsumerCallbacks = new ArrayList<>();

   public void addDetectionConsumerCallback(Consumer<List<InstantDetection>> callback)
   {
      detectionConsumerCallbacks.add(callback);
   }

   public YOLOv8DetectionExecutor(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator, BooleanSupplier annotatedImageDemanded)
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
      parameters = new SyncedYOLOv8ExecutorParameters(ros2Node, crdtInfo);
      parameters.setAvailableModels(availableModels.values());

      updateThread = new RepeatingTaskThread(getClass().getSimpleName() + "Updater", this::update);
      updateThread.setFrequencyLimit(10.0);
      updateThread.setDaemon(true);
      updateThread.startRepeating();

      annotatedImagePublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_ANNOTATED_IMAGE);
      annotatedImagePublishedThread = new RepeatingTaskThread("YOLOAnnotatedImagePublisher",
                                                              this::annotateAndPublishImage,
                                                              DefaultExceptionHandler.RUNTIME_EXCEPTION);
      annotatedImagePublishedThread.setDaemon(true);
      annotatedImagePublishedThread.startRepeating();
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
      List<YOLOv8InstantDetection> annotatedImageDetections = new ArrayList<>();

      // Get the image in BGR for annotation and running YOLO
      RawImage bgrImage = RawImageTools.convertColor(colorImage, PixelFormat.BGR8);

      YOLOv8Model yoloModel = availableModels.get(parameters.getModelToRun().getValue());
      if (yoloModel != null)
      {
         // Run YOLO to get results
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

         yoloResults.destroy();
      }

      // Process callbacks
      if (!yoloInstantDetections.isEmpty())
         detectionConsumerCallbacks.forEach(callback -> callback.accept(yoloInstantDetections));

      // Set new annotation notification
      if (annotationNotification.poll())
         annotationNotification.read().close();
      annotationNotification.set(new YOLOv8AnnotationRecord(bgrImage, annotatedImageDetections));

      bgrImage.release();
      colorImage.release();
      depthImage.release();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      updateThread.blockingKill();

      annotatedImagePublishedThread.kill();

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

   private record YOLOv8AnnotationRecord(RawImage colorImage, List<YOLOv8InstantDetection> detections)
   {
      YOLOv8AnnotationRecord(RawImage colorImage, List<YOLOv8InstantDetection> detections)
      {
         this.colorImage = colorImage.get();
         this.detections = detections;
      }

      public void close()
      {
         if (colorImage != null)
            colorImage.release();
         for (YOLOv8InstantDetection detection : detections)
            detection.destroy();
      }
   }

   private void annotateAndPublishImage()
   {
      annotationNotification.blockingPoll();
      YOLOv8AnnotationRecord annotationRecord = annotationNotification.read();
      List<YOLOv8InstantDetection> detectionsToAnnotate = annotationRecord.detections();
      RawImage colorImage = annotationRecord.colorImage();

      if (colorImage == null)
      {
         annotationRecord.close();
         return;
      }

      if (!annotatedImageDemanded.getAsBoolean())
      {
         annotationRecord.close();
         return;
      }

      Mat resultMat = new Mat();
      synchronized (annotatedImagePublishedThread)
      {
         YOLOv8Tools.annotateImage(colorImage.getCpuImageMat(), resultMat, detectionsToAnnotate);
      }

      Size size = new Size();
      opencv_imgproc.resize(resultMat, resultMat, size, 0.25, 0.25, opencv_imgproc.INTER_AREA);

      BytePointer annotatedImagePointer = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", resultMat, annotatedImagePointer); // for some reason using CUDAImageEncoder broke YOLO's CUDNN

      ImageMessage imageMessage = new ImageMessage();
      PerceptionMessageTools.packImageMessage(colorImage, annotatedImagePointer, CompressionType.JPEG, imageMessage);
      annotatedImagePublisher.publish(imageMessage);

      size.close();
      resultMat.close();
      annotationRecord.close();
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
